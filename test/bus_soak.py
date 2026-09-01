#!/usr/bin/env python3
"""Soak test for request-reply reliability on a crowded RMD CAN bus.

The point of this script is to make the failure the driver used to have visible and countable, so that a change
to the driver can be judged on numbers rather than on whether the sculpture looked jerky today.

It hammers every actuator on the bus with request-reply transactions, optionally while those same actuators are
broadcasting active replies, and reports per-actuator success and timeout counts, round-trip latency percentiles
and the driver's own view of the bus.

It deliberately runs against an unmodified build of the driver too, so that the same command can be used to take
a baseline first: anything the older build does not expose is detected and skipped rather than crashing. On an
older build the broadcasts have to be turned on separately, and mis-attributed replies show up in the 'protocol'
column, which is the number this is all about.

Nothing here commands motion. Every request is a read.

Typical use, on the Pi. Prefer --motors-json: it takes the actuators straight from the doll's own motor file and
excludes the Hitec servos and Moog SmartMotors sharing the address space, which --ids cannot do for you.

    # baseline, no broadcasts
    python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json --duration 30

    # the case that used to fall apart: every actuator broadcasting three frames at 10 Hz
    python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json \
        --duration 30 --active-reply 10

    # confirm the broadcasts are actually usable as a state feed
    python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json \
        --duration 30 --active-reply 10 --compare-cache

--ids takes an explicit set for one-off runs, and does not have to be contiguous:

    python3 test/bus_soak.py --interface can0 --ids 0x142,0x143,0x144,0x148,0x150 --duration 30

Active replies are disabled again on exit, including on Ctrl-C.
"""

import argparse
import json
import statistics
import sys
import time

import myactuator_rmd_py as rmd


# Command bytes that can be broadcast by the active reply feature (0xB6)
CMD_MULTI_TURN_ANGLE = 0x92
CMD_MOTOR_STATUS_1 = 0x9A
CMD_MOTOR_STATUS_2 = 0x9C

# Everything the fix adds. Probed rather than assumed so that a baseline run against an unmodified build still
# produces comparable numbers instead of an AttributeError on the first line.
HAS_STATISTICS = hasattr(rmd.Driver, "getStatistics")
HAS_RESPONSE_TIMEOUT = hasattr(rmd.Driver, "setResponseTimeout")
HAS_ACTIVE_REPLY = hasattr(rmd.ActuatorInterface, "enableActiveReply")
HAS_CACHE = hasattr(rmd.ActuatorInterface, "getCachedMultiTurnAngle")
TIMEOUT_EXCEPTION = getattr(rmd, "TimeoutException", None)
PROTOCOL_EXCEPTION = getattr(rmd, "ProtocolException", None)


def classify(exc):
    """Bucket an exception as 'timeout', 'protocol' or 'other'.

    Done by isinstance rather than by except clauses because the older build has no TimeoutException: there a
    missing reply surfaces as the socket read timing out, which lands in 'other' and is still perfectly legible
    next to a 'protocol' count.
    """
    if TIMEOUT_EXCEPTION is not None and isinstance(exc, TIMEOUT_EXCEPTION):
        return "timeout"
    if PROTOCOL_EXCEPTION is not None and isinstance(exc, PROTOCOL_EXCEPTION):
        return "protocol"
    return "other"


def parse_ids(spec):
    """Turn '0x141-0x154,0x160' or '1-20' into a sorted list of actuator ids in [1, 32].

    Both the raw actuator id and the CAN request address are accepted, since the two conventions are mixed
    throughout the surrounding tooling: anything at or above 0x140 is treated as an address and reduced.
    """
    ids = set()
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            low, high = chunk.split("-", 1)
            low, high = int(low, 0), int(high, 0)
        else:
            low = high = int(chunk, 0)
        if low > high:
            low, high = high, low
        for value in range(low, high + 1):
            ids.add(value - 0x140 if value >= 0x140 else value)
    bad = [i for i in ids if not 1 <= i <= 32]
    if bad:
        raise ValueError("actuator ids out of the admissible range [1, 32]: %s" % sorted(bad))
    return sorted(ids)


def is_rmd_address(address):
    """Whether a CAN address belongs to an RMD actuator driven by this library.

    Mirrors the dispatch in new-controller.py: at or above 0x160 the address is a Hitec servo, which speaks a
    different protocol on the same wire, and 128 to 144 is a Moog SmartMotor, which is not on CAN at all. Getting
    this wrong does not just skew the numbers, it fills the report with timeouts from devices that were never
    going to answer an RMD request.
    """
    if address >= 0x160:
        return False
    if 128 <= address <= 144:
        return False
    return True


def motors_from_json(path):
    """Read one of the doll-<name>/<name>-motors.json files.

    Returns the RMD actuators as {actuator_id: name}, plus a list of (address, name, reason) for everything that
    was left out, so the exclusions are visible rather than quietly assumed.
    """
    with open(path) as handle:
        payload = json.load(handle)
    addresses = payload["motor_addresses"]
    # Historically a list of hex strings, currently a dict of hex string to joint name
    if not isinstance(addresses, dict):
        addresses = {entry: "" for entry in addresses}

    motors = {}
    skipped = []
    for text, name in addresses.items():
        address = int(text, 16)
        if not is_rmd_address(address):
            reason = "Hitec servo" if address >= 0x160 else "Moog SmartMotor, not on CAN"
            skipped.append((address, name, reason))
            continue
        actuator_id = address - 0x140
        if not 1 <= actuator_id <= 32:
            skipped.append((address, name, "maps to actuator id %d, outside [1, 32]" % actuator_id))
            continue
        motors[actuator_id] = name
    return motors, skipped


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="can0", help="SocketCAN interface name (default: can0)")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--ids", help="actuator ids or CAN addresses; ranges and comma-separated sets both work, "
                                     "e.g. '0x142,0x143,0x148,0x150' or '0x141-0x154' or '1-20,25'")
    group.add_argument("--motors-json", help="path to a <doll>-motors.json; takes motor_addresses and keeps only "
                                             "the RMD actuators, reporting what it left out")
    parser.add_argument("--duration", type=float, default=20.0, help="seconds to run for (default: 20)")
    parser.add_argument("--response-timeout", type=float, default=None,
                        help="per-transaction reply timeout in seconds; leave unset to use the driver default")
    parser.add_argument("--active-reply", type=float, default=0.0, metavar="HZ",
                        help="enable 0x92/0x9A/0x9C broadcasts at this rate on every actuator (default: off)")
    parser.add_argument("--compare-cache", action="store_true",
                        help="also read the cached broadcast angle and report how far it lags the polled one")
    parser.add_argument("--command", choices=("angle", "status2"), default="angle",
                        help="which request to hammer the bus with (default: angle, i.e. 0x92)")
    args = parser.parse_args()

    skipped = []
    if args.motors_json:
        names, skipped = motors_from_json(args.motors_json)
        actuator_ids = sorted(names)
    else:
        actuator_ids = parse_ids(args.ids)
        names = {actuator_id: "" for actuator_id in actuator_ids}
    if not actuator_ids:
        parser.error("no actuator ids given")

    if args.compare_cache and not HAS_CACHE:
        parser.error("this build of myactuator_rmd_py has no response cache, so --compare-cache cannot run")
    if args.compare_cache and args.active_reply <= 0.0:
        parser.error("--compare-cache only means anything together with --active-reply")
    if args.response_timeout is not None and not HAS_RESPONSE_TIMEOUT:
        parser.error("this build of myactuator_rmd_py has no settable response timeout")

    driver = rmd.CanDriver(args.interface)
    if args.response_timeout is not None:
        driver.setResponseTimeout(args.response_timeout)

    actuators = {}
    for actuator_id in actuator_ids:
        actuators[actuator_id] = rmd.ActuatorInterface(driver, actuator_id)

    print("build            : %s" % ("with the transaction fix" if HAS_STATISTICS else "BASELINE, without the fix"))
    print("interface        : %s" % args.interface)
    print("actuators        : %d (%s)" % (len(actuator_ids),
                                          ", ".join(hex(0x140 + i) for i in actuator_ids)))
    for address, name, reason in skipped:
        print("  skipped        : %s %s (%s)" % (hex(address), name or "?", reason))
    if HAS_RESPONSE_TIMEOUT:
        print("response timeout : %.1f ms" % (driver.getResponseTimeout() * 1e3))
    if HAS_STATISTICS:
        recv_buffer_bytes = driver.getStatistics().recv_buffer_bytes
        print("receive buffer   : %d bytes" % recv_buffer_bytes)
        if recv_buffer_bytes < 1000000:
            print("  note: the kernel clamped the requested receive buffer to net.core.rmem_max.")
            print("        raise it with:  sudo sysctl -w net.core.rmem_max=8388608")

    broadcast_commands = (CMD_MULTI_TURN_ANGLE, CMD_MOTOR_STATUS_1, CMD_MOTOR_STATUS_2)
    broadcasts_enabled_here = False
    if args.active_reply > 0.0:
        expected_load = len(actuator_ids) * len(broadcast_commands) * args.active_reply
        if HAS_ACTIVE_REPLY:
            print("active replies   : %s at %.1f Hz -> %.0f frames/s on the bus"
                  % (", ".join(hex(c) for c in broadcast_commands), args.active_reply, expected_load))
            interval = 1.0 / args.active_reply
            for actuator in actuators.values():
                for command in broadcast_commands:
                    actuator.enableActiveReply(command, interval)
                time.sleep(0.01)
            broadcasts_enabled_here = True
        else:
            # The baseline build cannot send 0xB6 itself. Rather than quietly measuring a quiet bus and
            # reporting a clean result that means nothing, stop and say what to run first.
            parser.error(
                "this build of myactuator_rmd_py cannot enable active replies itself.\n"
                "Turn them on separately first, e.g. from the yogapi tree:\n"
                "    for id in %s; do python3 util/enable_active_replies_all.py $id %d; done\n"
                "then re-run this without --active-reply, and turn them off afterwards with\n"
                "    python3 util/disable_replies_all_motors.py"
                % (" ".join(hex(0x140 + i) for i in actuator_ids), int(args.active_reply)))
    else:
        print("active replies   : not enabled by this run")
    print()

    ok = {i: 0 for i in actuator_ids}
    timeouts = {i: 0 for i in actuator_ids}
    protocol_errors = {i: 0 for i in actuator_ids}
    other_errors = {i: 0 for i in actuator_ids}
    latencies = {i: [] for i in actuator_ids}
    cache_gaps = []
    first_error = {}

    if HAS_STATISTICS:
        driver.resetStatistics()
    started = time.monotonic()
    cycles = 0
    try:
        while time.monotonic() - started < args.duration:
            for actuator_id, actuator in actuators.items():
                began = time.monotonic()
                try:
                    if args.command == "angle":
                        value = actuator.getMultiTurnAngle()
                    else:
                        value = actuator.getMotorStatus2().shaft_angle
                    latencies[actuator_id].append((time.monotonic() - began) * 1e3)
                    ok[actuator_id] += 1
                    if args.compare_cache:
                        cached = actuator.getCachedMultiTurnAngle()
                        if cached is not None and args.command == "angle":
                            cache_gaps.append(abs(cached - value))
                except Exception as exc:  # noqa: BLE001 - the point is to survive and count whatever comes
                    bucket = classify(exc)
                    if bucket == "timeout":
                        timeouts[actuator_id] += 1
                    elif bucket == "protocol":
                        protocol_errors[actuator_id] += 1
                    else:
                        other_errors[actuator_id] += 1
                    first_error.setdefault(actuator_id, "%s: %s" % (type(exc).__name__, exc))
            cycles += 1
    except KeyboardInterrupt:
        print("interrupted, reporting what we have\n")
    finally:
        if broadcasts_enabled_here:
            for actuator in actuators.values():
                for command in broadcast_commands:
                    try:
                        actuator.disableActiveReply(command)
                    except Exception as exc:  # noqa: BLE001
                        print("  warning: could not disable broadcast %s: %s" % (hex(command), exc))
                time.sleep(0.01)

    elapsed = time.monotonic() - started
    total_ok = sum(ok.values())
    total_timeouts = sum(timeouts.values())
    total_protocol = sum(protocol_errors.values())
    total_other = sum(other_errors.values())
    total = total_ok + total_timeouts + total_protocol + total_other

    print("%-7s %-15s %8s %9s %9s %7s %9s %9s %9s" %
          ("id", "name", "ok", "timeout", "protocol", "other", "p50 ms", "p95 ms", "max ms"))
    for actuator_id in actuator_ids:
        samples = latencies[actuator_id]
        print("%-7s %-15s %8d %9d %9d %7d %9.2f %9.2f %9.2f" % (
            hex(0x140 + actuator_id), (names.get(actuator_id) or "")[:15],
            ok[actuator_id], timeouts[actuator_id],
            protocol_errors[actuator_id], other_errors[actuator_id],
            percentile(samples, 0.50), percentile(samples, 0.95),
            max(samples) if samples else float("nan")))

    print()
    print("cycles           : %d over %.1f s (%.1f Hz per actuator)" %
          (cycles, elapsed, cycles / elapsed if elapsed > 0 else 0.0))
    print("transactions     : %d total, %.4f%% failed" %
          (total, 100.0 * (total - total_ok) / total if total else 0.0))
    print("  timeouts       : %d" % total_timeouts)
    print("  protocol errors: %d   <- a reply arrived but did not match the request" % total_protocol)
    print("  other errors   : %d" % total_other)

    all_latencies = [value for samples in latencies.values() for value in samples]
    if all_latencies:
        print("latency          : p50 %.2f ms, p95 %.2f ms, p99 %.2f ms, max %.2f ms, mean %.2f ms" % (
            percentile(all_latencies, 0.50), percentile(all_latencies, 0.95),
            percentile(all_latencies, 0.99), max(all_latencies), statistics.mean(all_latencies)))

    if cache_gaps:
        print("cached vs polled : %d samples, median gap %.3f deg, max %.3f deg" %
              (len(cache_gaps), statistics.median(cache_gaps), max(cache_gaps)))

    if HAS_STATISTICS:
        print("driver           : %r" % (driver.getStatistics(),))

    if first_error:
        print("\nfirst error per actuator:")
        for actuator_id, message in sorted(first_error.items()):
            print("  %s %s  %s" % (hex(0x140 + actuator_id), names.get(actuator_id) or "", message))

    return 0 if total_ok == total else 1


if __name__ == "__main__":
    sys.exit(main())
