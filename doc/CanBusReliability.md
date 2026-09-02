# Request-reply reliability on a busy CAN bus

Notes for the `fix/can-transaction-correlation` branch. This describes what was wrong, what changed, and how to
check on hardware whether it helped.

## What was wrong

### 1. A reply was whatever frame arrived next

`CanNode::sendRecv` wrote the request and then read exactly one frame, returning its payload without checking
where it came from:

```cpp
write(can_send_id, request.getData());
can::Frame const frame {can::Node::read()};
return frame.getData();
```

The receive filter installed by `addId` is per socket, not per transaction, and it accumulates every actuator
registered on that driver. So on a bus with twenty actuators, the next frame to arrive was frequently a reply
addressed to a different actuator, or an active-reply broadcast, or the late reply to a transaction that had
already given up.

The only thing standing between that and silently wrong data was the `data[0] != C` check in the
`SingleMotorMessage` constructor, which turns a mismatch into a `ProtocolException`. That check is doing real
work — it is why this shows up as errors rather than as an actuator moving to the wrong angle — but it fires
*after* the wrong frame has already been consumed. It cannot detect a frame that came from the wrong actuator
but carried the right command byte, which is exactly what happens when several actuators are being polled with
the same command in a loop.

### 2. One lost reply desynchronised everything after it

Because the mismatched frame was consumed and the real reply was left queued, every subsequent transaction read
the *previous* transaction's answer. One dropped frame put the driver permanently one reply behind, and the only
way out was for the queue to drain on its own. This is the mechanism behind "it works with three motors and
falls apart with twenty": with three motors and no broadcasts the queue is almost always empty when a request
goes out, so the next frame really is the reply. Nothing about the code was correct at three motors — it was
just never given the chance to be wrong.

### 3. Active replies made the queue permanently non-empty

Active reply (command `0xB6`) makes an actuator broadcast `0x92` / `0x9A` / `0x9C` frames at a fixed rate. Those
broadcasts use the actuator's normal reply CAN id and its normal command byte, so nothing in the frame
distinguishes them from a solicited reply. Twenty actuators broadcasting three frames each at 10 Hz is 600
frames per second arriving on a socket that the driver only reads from once per transaction. The queue could
then never be empty, so the "next frame" was essentially never the reply, and the surplus accumulated until the
kernel started dropping frames.

That last part is worth stating plainly: the responses were not being lost on the wire. They were arriving, being
mis-attributed or overrun, and then discarded.

### 4. Everything else the socket layer was doing under load

- The kernel receive buffer was left at `net.core.rmem_default`, which holds well under a second of that traffic.
- `write()` treated `ENOBUFS` — the normal signal that the interface transmit queue is momentarily full — as a
  hard error, so commands were dropped exactly when the bus was busiest.
- Any CAN error frame, including a transient arbitration loss, was thrown from the middle of a transaction.
- The socket receive timeout was one second and applied per frame. A transaction with no reply coming stalled
  the whole control loop for a second; a transaction wading through broadcasts could restart that second on
  every frame.
- `can::Node` had a defaulted copy assignment and move constructor while owning a raw file descriptor, so any
  copy or move would have closed the same descriptor twice.

## What changed

### Replies are matched to their request

`sendRecv` now identifies the reply by the pair (reply CAN id, command byte) and skips anything else until it
finds it or the deadline expires. On the `0x400`/`0x500` motion mode range there is no command byte to compare —
`data[0]` there is packed position data — so matching is on the CAN id alone, which is sufficient because motion
mode replies and active-reply broadcasts do not share an id range. Which rule applies is decided at compile time
by `ResponseMatching<SEND_ID_OFFSET>`.

### The queue is cleared before each request, not after

`sendRecv` drains everything already queued *before* sending. Anything sitting there predates the request and so
cannot be its reply. This is what stops one lost reply from cascading into every transaction that follows.

### Skipped frames are kept rather than discarded

Every frame that is not the reply is stored as the most recent `CachedResponse` for its (actuator, command)
pair. This turns the active-reply broadcasts from interference into a state feed:

```python
actuator.enableActiveReply(0x92, 0.1)      # broadcast the multi-turn angle at 10 Hz
...
angle = actuator.getCachedMultiTurnAngle()  # no bus traffic at all
cached = actuator.getCachedResponse(0x92)
if cached is not None and cached.getAgeSeconds() < 0.5:
    ...                                     # check the age; a silent actuator is not a stationary one
```

For a sculpture that wants current position and torque from twenty actuators, this is the difference between
twenty request-reply round trips per cycle and none. `driver.pollTelemetry()` collects broadcasts during idle
periods when no transaction is happening to do it implicitly.

### One deadline per transaction

The per-transaction timeout is separate from the socket receive timeout and defaults to 100 ms, against the
previous effective one second. It bounds the whole exchange rather than each frame, so skipping over a burst of
broadcasts cannot extend the wait. A missing reply now raises `TimeoutException` (new, distinct from
`ProtocolException`, which continues to mean a reply arrived but could not be parsed), so a control loop can tell
"this actuator is not answering" from "this data is corrupt" and retry accordingly.

`driver.setResponseTimeout(seconds)` changes it. If a ROM-writing command such as `setEncoderZero` or a
persistent `setControllerGains` starts timing out, that is the knob.

### Socket-level fixes

- The receive buffer is raised to 4 MB at construction. The kernel silently clamps this to `net.core.rmem_max`
  unless the process has `CAP_NET_ADMIN`, so check what was actually granted via
  `driver.getStatistics().recv_buffer_bytes` and raise the ceiling if needed:
  `sudo sysctl -w net.core.rmem_max=8388608` (`/etc/sysctl.d/` to persist).
- `write()` waits for the transmit queue to drain and retries within the send timeout instead of failing on
  `ENOBUFS`.
- Transient CAN error frames are counted and stepped over rather than aborting a transaction. `BusOffError` and
  `ControllerRestartedError` are still thrown, because those are not transient.
- `sendRecv` and `send` hold a mutex for the whole exchange, so two threads sharing a driver cannot end up
  reading each other's replies.
- `can::Node` copying is forbidden and moving transfers the descriptor properly.

### Bus statistics

`driver.getStatistics()` reports frames received, replies matched, frames cached, timeouts, absorbed bus errors,
the granted receive buffer size and the last bus error. A rising timeout count alongside a healthy cache count
means replies are being crowded out; a receive buffer far below 4 MB points at `net.core.rmem_max` rather than
at the bus.

### Active reply from the library

`enableActiveReply(command, interval_seconds)` / `disableActiveReply(command)` implement `0xB6`, so the broadcast
configuration no longer needs a second python-can bus alongside the driver. It is sent without waiting for an
acknowledgement, because the actuator's answer to it is itself indistinguishable from the broadcasts it is being
asked to start.

## Building and installing

Two pre-existing packaging bugs made it easy to build this branch and then go on running the old binary. Both
are fixed here, but they are worth knowing about because the symptom is silent.

`install(TARGETS myactuator_rmd_py DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})` relied on
`PYTHON_INSTALL_DIR`, which is set by `ament_cmake_python` and by nothing else. Outside ROS it expanded to
nothing, so the destination became the absolute path `/myactuator_rmd`. `sudo make install` wrote the module to
the filesystem root, reported success, and `import myactuator_rmd_py` carried on resolving to whatever older
copy was in site-packages. The non-ROS branch now installs to `Python3_SITEARCH` and prints where at configure
time.

`pip install .` failed with "could not build wheel" because there was no `pyproject.toml`, so pip built in an
isolated environment containing no pybind11 and `find_package(pybind11 CONFIG REQUIRED)` failed a long way above
the visible error. There is a `pyproject.toml` now, and `setup.py` passes `pybind11_DIR` through to CMake.

Either route works:

```bash
pip3 install .
```

```bash
mkdir -p build && cd build && cmake .. -D PYTHON_BINDINGS=ON && make -j$(nproc) && sudo make install
```

Watch for the `Python bindings will be installed to:` line during configure, and check afterwards that the
module Python actually imports is the one just built:

```bash
cd /tmp && python3 -c "import myactuator_rmd_py as rmd, os, time; print(rmd.__file__, time.ctime(os.path.getmtime(rmd.__file__)))"
```

Note that `myactuator_rmd_py` links against `libmyactuator_rmd.so` and the two must come from the same build.
Installing one without the other leaves a mismatch that links cleanly and misbehaves at runtime, because this
change alters the size of `can::Node` and the vtable of `Driver`.

## Checking it on hardware

`test/bus_soak.py` hammers every actuator with request-reply transactions and reports per-actuator success and
timeout counts, latency percentiles and the driver statistics.

Point it at the doll's own motor file rather than an id range. Addresses on an RMD bus are rarely contiguous,
and the same address space carries Hitec servos (at or above 0x160) and Moog SmartMotors (128 to 144) that are
not RMD actuators at all -- included by mistake they contribute nothing but timeouts. `--motors-json` applies the
same classification `new-controller.py` uses and prints what it excluded.

```bash
python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json --duration 30
```

```bash
python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json --duration 30 --active-reply 10
```

```bash
python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json --duration 30 --active-reply 10 --compare-cache
```

`--ids` still takes an explicit set for one-off runs, and does not have to be contiguous:

```bash
python3 test/bus_soak.py --interface can0 --ids 0x142,0x143,0x144,0x148,0x150 --duration 30
```

Active replies are disabled again on exit, including on Ctrl-C.

The script probes for everything the fix adds, so the same commands also run against an unmodified `main`
build and give a comparable baseline. On `main` it cannot send `0xB6` itself, so it will tell you to enable the
broadcasts with the existing `yogapi/util/enable_active_replies_all.py` and re-run without `--active-reply`.

The run to compare against is the same command on `main`: the second one, with broadcasts enabled, is where the
old behaviour should show a large `protocol` count and where this branch should show neither protocol errors nor
timeouts. `--compare-cache` reports how far the broadcast angle sits from the polled one, which is a check that
the cached feed is genuinely usable rather than merely present.

## Active reply and request-reply are mutually exclusive

Protocol section 2.35.1 is explicit: "If an active reply command is set, the motor will not reply after receiving
the command." Not merely the commands being broadcast -- anything. Confirmed on hardware: polling 0x9D against
nine actuators in active reply mode times out 198 times out of 198, while frames keep arriving and being cached.

Two consequences worth being clear about, because they look identical from the application and only one of them
is a software problem:

- An actuator **in** active reply mode never answers a 0xA4 position command. The feedback frame is not being
  lost, it is not being sent. No driver change can recover it; the state has to come from the broadcasts, via
  the response cache.
- An actuator **not** in active reply mode, on a bus where others are broadcasting, does answer normally. This is
  where the old driver failed: it read one frame and returned it unchecked, so a broadcast that arrived first was
  consumed in place of the reply, which was then left queued to corrupt the following transaction. That is the
  bug this branch fixes.

The mixed configuration is the production one and the only one worth measuring, hence `--broadcast-ids`:

```bash
python3 test/bus_soak.py --interface can0 --motors-json ../yogapi/doll-yogapi/yogapi-motors.json --duration 30 --active-reply 10 --broadcast-ids 0x144,0x145
```

It is also worth asking whether active reply earns its place at all. A 0xA4 reply already carries temperature,
torque current, shaft speed and shaft angle -- the same MotorStatus2 payload the 0x9C broadcast carries -- fresh
and correlated to the command that produced it. Polling twenty actuators costs roughly 3,000 frames/s, about 44%
of a 1 Mbps bus. Active reply reduces that to 20 frames/s but gives up command acknowledgement entirely.

## Commanding an actuator that is broadcasting

Because an actuator in active reply mode answers nothing, every ordinary motion command against one waits out
the full response timeout and then throws. The `...NoReply` variants send without waiting:

```python
actuator.sendPositionAbsoluteSetpointNoReply(position_deg, max_speed_dps)
actuator.sendVelocitySetpointNoReply(speed_dps)
actuator.sendForcePositionSetpointNoReply(position_deg, max_speed_dps, max_torque)
actuator.sendCurrentSetpointNoReply(current_a)
motion_actuator.sendMotionModeSetpointNoReply(p_des, v_des, kp, kd, t_ff_percent)
```

There is no acknowledgement, so nothing confirms the actuator acted. Its state comes from the broadcasts, and
the age of a cached value is the only evidence it is still alive.

Whether active reply also suppresses the 0x500 motion mode reply is not stated in the protocol and has not been
checked on hardware. If `sendMotionModeSetpoint` keeps working against a broadcasting actuator, it does not.

### Reading state back

```python
cached = actuator.getCachedResponse(0x92)
if cached is not None and cached.getAgeSeconds() < max_age:
    angle_deg = actuator.getCachedMultiTurnAngle()
```

Two things to hold on to. `None` means never heard from, which is not the same as stale -- an actuator that
stops broadcasting keeps returning its last value indefinitely, and a frozen angle is indistinguishable from a
stationary joint unless the age is checked. And use the 0x92 cache for position: `MotorStatus2.shaft_angle` from
0x9C is a 1 degree integer, where 0x92 carries 0.01 degree resolution.

Broadcasting one command rather than three matters more than it looks. The actuator emits one frame per interval
and cycles through whatever set is configured, so three commands at 10 Hz means any individual one arrives every
300 ms, not every 100 ms.

## What this does not change

The driver is still synchronous: a transaction blocks its caller until the reply arrives. That is fine while the
application keeps talking to the bus, because every transaction drains the queue on the way past. If the
application goes quiet for long enough that broadcasts overrun even the enlarged receive buffer, frames are still
lost — call `driver.pollTelemetry()` from the idle path to avoid that. A dedicated receive thread would remove
the caveat entirely and is the obvious next step if the idle path turns out to be awkward to reach.
