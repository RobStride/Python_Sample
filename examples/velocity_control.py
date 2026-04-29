"""
Velocity Mode Example.
速度模式示例

This example demonstrates how to drive a Robstride motor in velocity mode
(run_mode = 2). In this mode the host writes a target speed (`spd_ref`,
0x700A) and the drive's internal velocity loop accelerates / decelerates the
motor towards that speed using the configured current limit (`limit_cur`,
0x7018) and acceleration (`acc_rad`, 0x7022).

Reference:
    RS01 使用说明书 section 4.3.3.

Workflow per the manual:
    disable -> set run_mode=2 -> enable
        -> write limit_cur -> write acc_rad -> write spd_ref ...
        -> write spd_ref=0 -> disable

CAUTION:
    The motor will spin when this script runs. Make sure the output shaft
    is free to rotate or safely loaded.
"""

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

CURRENT_LIMIT = 5.0
"""Phase current limit (A) used by the velocity loop."""

ACCELERATION = 10.0
"""Velocity-mode acceleration (rad/s^2)."""

VELOCITY_PROFILE_RAD_S = (3.0, -3.0, 1.5, 0.0)
"""Sequence of target velocities (rad/s) the demo will run through."""

DWELL_SECONDS = 2.0
"""How long to hold each target velocity before issuing the next."""

SAMPLE_PERIOD_SECONDS = 0.2
"""How often to read back the actual velocity for printing."""


def main() -> None:
    bus = RobstrideBus(
        channel=CHANNEL,
        motors={MOTOR_NAME: Motor(id=MOTOR_ID, model=MOTOR_MODEL)},
    )
    bus.connect()

    try:
        # Mode switching must happen while the motor is disabled.
        # 切换运行模式必须在电机失能状态下进行
        bus.disable(MOTOR_NAME)
        bus.set_run_mode(MOTOR_NAME, 2)
        bus.enable(MOTOR_NAME)

        for target in VELOCITY_PROFILE_RAD_S:
            print(f"\n-> target velocity = {target:+.2f} rad/s "
                  f"(limit_cur={CURRENT_LIMIT} A, acc={ACCELERATION} rad/s^2)")
            bus.set_target_velocity(
                motor=MOTOR_NAME,
                velocity=target,
                acceleration=ACCELERATION,
                current_limit=CURRENT_LIMIT,
            )

            # Sample the actual velocity periodically while the drive
            # accelerates / holds the new setpoint.
            t_end = time.perf_counter() + DWELL_SECONDS
            while time.perf_counter() < t_end:
                actual_velocity = bus.read(MOTOR_NAME, ParameterType.MECHANICAL_VELOCITY)
                actual_iq = bus.read(MOTOR_NAME, ParameterType.IQ_FILTERED)
                print(f"   v={actual_velocity:+.3f} rad/s  iq={actual_iq:+.3f} A")
                time.sleep(SAMPLE_PERIOD_SECONDS)

    finally:
        # Bring the motor to a stop before disabling so it does not coast.
        try:
            bus.set_target_velocity(MOTOR_NAME, velocity=0.0)
            time.sleep(0.5)
        finally:
            try:
                bus.disable(MOTOR_NAME)
            finally:
                bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
