#!/usr/bin/env python3
"""Soak test for request-reply reliability on a crowded RMD CAN bus.

The point of this script is to make the failure the driver used to have visible and countable, so that a change
to the driver can be judged on numbers rather than on whether the sculpture looked jerky today.

It hammers every actuator on the bus with request-reply transactions, optionally while those same actuators are
broadcasting active replies, and reports per-actuator success and timeout counts, round-trip latency percentiles
and the driver's own view of the bus.

Typical use, on the Pi:

    # baseline, no broadcasts
    python3 test/bus_soak.py --interface can0 --ids 0x141-0x154 --duration 30

    # the case that used to fall apart: every actuator broadcasting three frames at 10 Hz
    python3 test/bus_soak.py --interface can0 --ids 0x141-0x154 --duration 30 --active-reply 10

    # confirm the broadcasts are actually usable as a state feed
    python3 test/bus_soak.py --interface can0 --ids 0x141-0x154 --duration 30 --active-reply 10 --compare-cache

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


def ids_from_motors_json(path):
    """Read the motor_addresses list out of one of the doll-<name>/<name>-motors.json files."""
    with open(path) as handle:
        payload = json.load(handle)
    return parse_ids(",".join(payload["motor_addresses"]))


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
    group.add_argument("--ids", help="actuator ids or CAN addresses, e.g. '0x141-0x154' or '1-20,25'")
    group.add_argument("--motors-json", help="path to a <doll>-motors.json to take motor_addresses from")
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

    if args.motors_json:
        actuator_ids = ids_from_motors_json(args.motors_json)
    else:
        actuator_ids = parse_ids(args.ids)
    if not actuator_ids:
        parser.error("no actuator ids given")

    if args.compare_cache and args.active_reply <= 0.0:
        parser.error("--compare-cache only means anything together with --active-reply")

    driver = rmd.CanDriver(args.interface)
    if args.response_timeout is not None:
        driver.setResponseTimeout(args.response_timeout)

    actuators = {}
    for actuator_id in actuator_ids:
        actuators[actuator_id] = rmd.ActuatorInterface(driver, actuator_id)

    print("interface        : %s" % args.interface)
    print("actuators        : %d (%s)" % (len(actuator_ids),
                                          ", ".join(hex(0x140 + i) for i in actuator_ids)))
    print("response timeout : %.1f ms" % (driver.getResponseTimeout() * 1e3))
    print("receive buffer   : %d bytes" % driver.getStatistics().recv_buffer_bytes)
    if driver.getStatistics().recv_buffer_bytes < 1_000_000:
        print("  note: the kernel clamped the requested receive buffer to net.core.rmem_max.")
        print("        raise it with:  sudo sysctl -w net.core.rmem_max=8388608")

    broadcast_commands = (CMD_MULTI_TURN_ANGLE, CMD_MOTOR_STATUS_1, CMD_MOTOR_STATUS_2)
    if args.active_reply > 0.0:
        interval = 1.0 / args.active_reply
        print("active replies   : %s at %.1f Hz -> %.0f frames/s on the bus"
              % (", ".join(hex(c) for c in broadcast_commands), args.active_reply,
                 len(actuator_ids) * len(broadcast_commands) * args.active_reply))
        for actuator_id, actuator in actuators.items():
            for command in broadcast_commands:
                actuator.enableActiveReply(command, interval)
            time.sleep(0.01)
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
                except rmd.TimeoutException as exc:
                    timeouts[actuator_id] += 1
                    first_error.setdefault(actuator_id, str(exc))
                except rmd.ProtocolException as exc:
                    protocol_errors[actuator_id] += 1
                    first_error.setdefault(actuator_id, str(exc))
                except Exception as exc:  # noqa: BLE001 - the point is to survive and count whatever comes
                    other_errors[actuator_id] += 1
                    first_error.setdefault(actuator_id, "%s: %s" % (type(exc).__name__, exc))
            cycles += 1
    except KeyboardInterrupt:
        print("interrupted, reporting what we have\n")
    finally:
        if args.active_reply > 0.0:
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

    print("%-8s %8s %9s %9s %7s %9s %9s %9s" %
          ("id", "ok", "timeout", "protocol", "other", "p50 ms", "p95 ms", "max ms"))
    for actuator_id in actuator_ids:
        samples = latencies[actuator_id]
        print("%-8s %8d %9d %9d %7d %9.2f %9.2f %9.2f" % (
            hex(0x140 + actuator_id), ok[actuator_id], timeouts[actuator_id],
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

    print("driver           : %r" % (driver.getStatistics(),))

    if first_error:
        print("\nfirst error per actuator:")
        for actuator_id, message in sorted(first_error.items()):
            print("  %s  %s" % (hex(0x140 + actuator_id), message))

    return 0 if total_ok == total else 1


if __name__ == "__main__":
    sys.exit(main())
