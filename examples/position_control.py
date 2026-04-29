"""
Position Mode Example.
位置模式示例

This example demonstrates how to drive a Robstride motor in position mode.
Two flavours are supported by the controller:

* PP  - Profile Position mode  (run_mode = 1).
        The drive plans a trapezoidal profile from the current position to
        the target position using the configured maximum velocity (vel_max)
        and acceleration (acc_set). Use this when you want to issue discrete
        "go-to-position" commands and let the drive plan the motion.

* CSP - Cyclic Synchronous Position mode  (run_mode = 5).
        The host streams target positions at a high rate, and the drive
        clamps the resulting velocity below the configured limit (limit_spd).
        Use this when the host runs its own trajectory planner.

Reference:
    RS01 使用说明书 sections 4.3.4 (CSP) and 4.3.5 (PP).

CAUTION:
    The motor will move when this script runs. Keep clear of the output
    shaft, and make sure the configured travel range is mechanically safe.
"""

import math
import time

from robstride_dynamics import Motor, ParameterType, RobstrideBus


CHANNEL = "can0"
"""SocketCAN channel that the motor is connected to."""

MOTOR_NAME = "joint_1"
"""Friendly name used to refer to the motor in the bus."""

MOTOR_ID = 0x01
"""CAN id of the motor (factory default for most Robstride actuators)."""

MOTOR_MODEL = "rs-01"
"""Model string used to look up the scaling tables in `table.py`."""


# ---------------------------------------------------------------------------
# PP (profile position) demo settings
# ---------------------------------------------------------------------------

PP_VELOCITY_MAX = 5.0
"""PP profile maximum velocity, in rad/s."""

PP_ACCELERATION = 10.0
"""PP profile acceleration, in rad/s^2."""

PP_WAYPOINTS_RAD = (1.0, -1.0, 0.5, 0.0)
"""Sequence of absolute target positions (relative to the start position).
PP 模式下要走的几个目标点，相对于上电时的当前位置。"""

PP_DWELL_SECONDS = 1.5
"""How long to wait at each PP waypoint before issuing the next one."""


# ---------------------------------------------------------------------------
# CSP (cyclic synchronous position) demo settings
# ---------------------------------------------------------------------------

CSP_VELOCITY_LIMIT = 8.0
"""CSP velocity limit, in rad/s."""

CSP_FREQUENCY_HZ = 100.0
"""Streaming rate for CSP target positions."""

CSP_DURATION_SECONDS = 6.0
"""Total duration of the CSP sinusoid demo."""

CSP_AMPLITUDE_RAD = 0.5
"""Amplitude of the CSP sinusoidal trajectory, in rad."""

CSP_SINE_HZ = 0.5
"""Frequency of the CSP sinusoid, in Hz."""


def read_current_position(bus: RobstrideBus) -> float:
    """Read the current mechanical position from the motor."""
    return bus.read(MOTOR_NAME, ParameterType.MECHANICAL_POSITION)


def run_pp_demo(bus: RobstrideBus, start_position: float) -> None:
    """Drive the motor through a few absolute waypoints using PP mode."""
    print("\n--- PP (profile position) demo ---")

    # Mode switching must happen while the motor is disabled.
    # 切换运行模式必须在电机失能状态下进行
    bus.disable(MOTOR_NAME)
    bus.set_run_mode(MOTOR_NAME, 1)
    bus.enable(MOTOR_NAME)

    for waypoint in PP_WAYPOINTS_RAD:
        target = start_position + waypoint
        print(f"PP -> target = {target:+.3f} rad "
              f"(vel_max={PP_VELOCITY_MAX} rad/s, acc={PP_ACCELERATION} rad/s^2)")

        bus.move_to_position_pp(
            motor=MOTOR_NAME,
            position=target,
            velocity_max=PP_VELOCITY_MAX,
            acceleration=PP_ACCELERATION,
        )

        # The drive runs the profile autonomously; we just dwell here long
        # enough for the motion to complete and then read back the result.
        time.sleep(PP_DWELL_SECONDS)
        position = read_current_position(bus)
        print(f"   reached  = {position:+.3f} rad  (error={position - target:+.4f})")


def run_csp_demo(bus: RobstrideBus, start_position: float) -> None:
    """Stream a sinusoidal position reference using CSP mode."""
    print("\n--- CSP (cyclic synchronous position) demo ---")

    bus.disable(MOTOR_NAME)
    bus.set_run_mode(MOTOR_NAME, 5)
    bus.enable(MOTOR_NAME)

    # Set the velocity limit once; subsequent calls only update loc_ref.
    bus.move_to_position_csp(
        motor=MOTOR_NAME,
        position=start_position,
        velocity_limit=CSP_VELOCITY_LIMIT,
    )

    period = 1.0 / CSP_FREQUENCY_HZ
    loop_start = time.perf_counter()
    next_tick = loop_start

    while True:
        elapsed = time.perf_counter() - loop_start
        if elapsed >= CSP_DURATION_SECONDS:
            break

        target = start_position + CSP_AMPLITUDE_RAD * math.sin(
            2.0 * math.pi * CSP_SINE_HZ * elapsed
        )

        bus.move_to_position_csp(motor=MOTOR_NAME, position=target)

        if int(elapsed * CSP_FREQUENCY_HZ) % int(CSP_FREQUENCY_HZ / 5) == 0:
            # Print every 0.2s to avoid flooding the terminal.
            position = read_current_position(bus)
            print(
                f"t={elapsed:5.2f}s  "
                f"pos_ref={target:+.3f} rad  pos={position:+.3f} rad  "
                f"err={position - target:+.4f}"
            )

        next_tick += period
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick = time.perf_counter()


def main() -> None:
    bus = RobstrideBus(
        channel=CHANNEL,
        motors={MOTOR_NAME: Motor(id=MOTOR_ID, model=MOTOR_MODEL)},
    )
    bus.connect()

    try:
        # Read the current position once and use it as the reference for
        # both demos so the motor never makes a large unexpected jump.
        # 读取当前位置作为后续轨迹基准，避免上电瞬间突跳
        start_position = read_current_position(bus)
        print(f"Start position: {start_position:+.4f} rad")

        run_pp_demo(bus, start_position)
        run_csp_demo(bus, start_position)

    finally:
        try:
            bus.disable(MOTOR_NAME)
        finally:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
