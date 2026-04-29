# Robstride Dynamics Python SDK

Python API for the RobStride series of integrated CAN servo motors (RS-00 ~
RS-06). The SDK speaks the manufacturer's private CAN protocol over a Linux
SocketCAN interface and exposes a small, idiomatic Python class
(`RobstrideBus`) for reading parameters, writing parameters, and controlling
motors in any of the four supported run modes.

## Features

- Full coverage of the four control modes documented in the RS01 manual:
  - **Operation control mode** (MIT, `run_mode = 0`) — single-frame impedance
    control with feed-forward velocity / torque.
  - **Position mode (PP)** (`run_mode = 1`) — profile position with
    `vel_max` / `acc_set` / `loc_ref`.
  - **Position mode (CSP)** (`run_mode = 5`) — cyclic synchronous position
    with `limit_spd` / `loc_ref`.
  - **Velocity mode** (`run_mode = 2`) — `limit_cur` / `acc_rad` / `spd_ref`.
  - **Current mode** (`run_mode = 3`) — direct `iq_ref`.
- Bus-level conveniences: `enable`, `disable`, `set_run_mode`, parameter
  `read` / `write`, `ping_by_id`, `scan_channel`, `read_id`, `write_id`.
- Optional per-motor `calibration` (`direction`, `homing_offset`) is applied
  consistently across operation, position, velocity, and current control
  helpers, so user-frame setpoints are decoupled from raw motor frame.
- Detailed status / fault decoding from the type-2 feedback frame (uncalibrated,
  stall, encoder fault, over-temperature, over-current, under-voltage…).

## Installing

### From PyPI

```bash
pip install robstride-dynamics
```

### From source (editable)

```bash
git clone https://github.com/ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK.git
cd Robstride-Dynamics-Python-SDK
pip install -e .
```

The SDK requires Python ≥ 3.10 and depends only on `numpy`, `python-can`,
and `tqdm`.

## Hardware setup

1. Wire the motor's CAN bus to a SocketCAN-compatible adapter (e.g. the
   official RobStride USB-CAN module). Make sure the 120 Ω terminator is
   enabled on at least one end of the bus.
2. Bring up the CAN interface on Linux at the motor's baud rate (1 Mbps by
   default):
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```
3. Power the motor and confirm the controller LED is steady.
4. (Optional) Sniff the bus to confirm wiring:
   ```bash
   candump can0
   ```

If you do not know the motor's CAN id, use `RobstrideBus.scan_channel`:

```python
from robstride_dynamics import RobstrideBus

print(RobstrideBus.scan_channel("can0"))
# {1: [1, <uuid>]}
```

## Quick start

```python
from robstride_dynamics import Motor, ParameterType, RobstrideBus

bus = RobstrideBus(
    channel="can0",
    motors={"joint_1": Motor(id=0x01, model="rs-01")},
)
bus.connect()

try:
    # Read a parameter (e.g. mechanical position in rad)
    pos = bus.read("joint_1", ParameterType.MECHANICAL_POSITION)
    print(f"current position: {pos:+.3f} rad")

    # Switch to operation control mode and enable torque
    bus.disable("joint_1")
    bus.set_run_mode("joint_1", 0)
    bus.enable("joint_1")

    # Send one MIT control frame (hold current position with light impedance)
    bus.write_operation_frame(
        motor="joint_1",
        position=pos,
        kp=10.0, kd=1.0,
        velocity=0.0, torque=0.0,
    )
    print(bus.read_operation_frame("joint_1"))

finally:
    bus.disable("joint_1")
    bus.disconnect(disable_torque=False)
```

## Examples

The `examples/` folder contains one runnable script per control mode.
All scripts share the same constants block (`CHANNEL`, `MOTOR_NAME`,
`MOTOR_ID`, `MOTOR_MODEL`) at the top, edit those to match your hardware
before running.

| Script | Mode | run_mode | What it does |
| --- | --- | --- | --- |
| `examples/operation_control.py` | Operation control (MIT) | 0 | Tracks a sinusoidal position reference at 200 Hz using `write_operation_frame` with feed-forward velocity. |
| `examples/position_control.py`  | Position (PP) + (CSP)   | 1, 5 | Walks through a few PP waypoints with a trapezoidal profile, then streams a CSP sinusoid at 100 Hz. |
| `examples/velocity_control.py`  | Velocity                | 2 | Steps through a list of target velocities, holding each for a fixed time and printing actual `v` / `iq`. |
| `examples/current_control.py`   | Current                 | 3 | Steps through a small ±Iq profile, printing actual `v` / `iq` / `tau`. Use a free or safely-loaded shaft. |

Run any example with:

```bash
python examples/operation_control.py
```

Each script follows the same safety pattern:

```
disable -> set_run_mode -> enable -> control loop -> write zero setpoint -> disable -> disconnect
```

## API overview

### Bus lifecycle

- `RobstrideBus(channel, motors, calibration=None, bitrate=1_000_000)`
- `bus.connect()`, `bus.disconnect(disable_torque=True)`
- `bus.is_connected`, `bus.scan_channel(channel)`, `bus.ping_by_id(id)`

### Per-motor state

- `bus.enable(motor)`, `bus.disable(motor)`
- `bus.set_run_mode(motor, run_mode)` — `0` MIT, `1` PP, `2` velocity,
  `3` current, `5` CSP
- `bus.read_id(motor)`, `bus.write_id(motor, new_id)`

### Parameter I/O

- `bus.read(motor, ParameterType.X)` → value
- `bus.write(motor, ParameterType.X, value)`

`ParameterType` exposes every documented index (e.g. `MODE`,
`MECHANICAL_POSITION`, `MECHANICAL_VELOCITY`, `IQ_FILTERED`,
`POSITION_TARGET`, `VELOCITY_TARGET`, `IQ_TARGET`, `PP_VELOCITY_MAX`,
`PP_ACCELERATION_TARGET`, `VELOCITY_LIMIT`, `CURRENT_LIMIT`, …).

### Control helpers

| Method | Mode |
| --- | --- |
| `write_operation_frame(motor, position, kp, kd, velocity=0, torque=0)` | Operation control (MIT) |
| `read_operation_frame(motor)` → `(pos, vel, torque, temp)` | Operation control (MIT) |
| `move_to_position_pp(motor, position, velocity_max=None, acceleration=None)` | PP |
| `move_to_position_csp(motor, position, velocity_limit=None)` | CSP |
| `set_target_velocity(motor, velocity, acceleration=None, current_limit=None)` | Velocity |
| `set_target_current(motor, iq)` | Current |

All `*_position*` / `*_velocity` / `*_current` helpers respect the optional
`calibration` dict and convert from user frame to motor frame internally.

## Calibration (optional)

`RobstrideBus` accepts a `calibration` dict so user-frame setpoints can be
expressed regardless of how the motor is mounted:

```python
calibration = {
    "joint_1": {
        "direction": -1,        # flip rotation
        "homing_offset": 1.5,   # rad subtracted from user position
    },
}

bus = RobstrideBus(
    channel="can0",
    motors={"joint_1": Motor(id=0x01, model="rs-01")},
    calibration=calibration,
)
```

When `calibration` is provided, all control helpers automatically apply
`raw = user * direction + homing_offset` on the way out and the inverse on
the way back, so the application code only ever sees user-frame values.

## Safety notes

- Always run `disable` (or rely on `disconnect(disable_torque=True)`) before
  exiting your script. Both are idempotent.
- Mode switches must happen with the motor disabled. The example scripts and
  helper methods follow this rule; if you mix raw `bus.write()` calls with
  control helpers, do the same.
- In current mode there is no velocity feedback loop — even small Iq values
  will accelerate a free shaft very quickly. Always start small and write
  `iq=0` before disabling.
- If you observe `WARNING: Received no response from the motor`, check the
  CAN id, baud rate, terminator, and 24 V/36 V/48 V supply. The motor only
  sends type-2 feedback frames in response to host commands unless active
  reporting (type 24) has been enabled.

## Documentation references

The protocol implementation follows the RobStride RS01 user manual. Key
sections:

- §3.4 — Upper-computer demos for each control mode.
- §4.1 — Private communication protocol (frame layout for types 0/1/2/3/4/
  17/18/21/22/23/24/25).
- §4.3 — Per-mode workflow (operation control, current, velocity, position
  CSP, position PP, stop).
- §4.4 — C reference snippets for enable / operation control / stop /
  parameter write.

## License

MIT — see [LICENSE](LICENSE).
