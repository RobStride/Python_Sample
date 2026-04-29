"""
Current Mode Example.
电流模式示例

This example demonstrates how to drive a Robstride motor in current mode
(run_mode = 3). In this mode the host writes a target Iq current
(`iq_ref`, 0x7006) and the drive's current loop tracks it. The motor's
torque is approximately `Kt * iq_ref` at the rotor (Kt = 1.22 N.m/Arms for
RS01) and `Kt * iq_ref * gear_ratio` at the output shaft.

Reference:
    RS01 使用说明书 section 4.3.2.

Workflow per the manual:
    disable -> set run_mode=3 -> enable
        -> write iq_ref ...
        -> write iq_ref=0 -> disable

CAUTION:
    Current mode has NO velocity feedback loop. With no/low load, even a
    small Iq will accelerate the motor very quickly. Always:
        1. Start from very small currents (a few hundred mA).
        2. Write iq=0 before disabling.
        3. Make sure the output shaft is free to rotate or safely loaded.
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

# Sequence of (iq_ref [A], duration [s]) pairs to step through.
# Keep currents small for a free-spinning bench setup.
CURRENT_PROFILE = (
    (+0.3, 1.5),
    (0.0, 1.0),
    (-0.3, 1.5),
    (0.0, 1.0),
)

SAMPLE_PERIOD_SECONDS = 0.1
"""How often to read back the actual state for printing."""


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
        bus.set_run_mode(MOTOR_NAME, 3)
        bus.enable(MOTOR_NAME)

        for iq_ref, duration in CURRENT_PROFILE:
            print(f"\n-> iq_ref = {iq_ref:+.3f} A  for {duration:.1f} s")
            bus.set_target_current(MOTOR_NAME, iq=iq_ref)

            t_end = time.perf_counter() + duration
            while time.perf_counter() < t_end:
                velocity = bus.read(MOTOR_NAME, ParameterType.MECHANICAL_VELOCITY)
                iq_fdb = bus.read(MOTOR_NAME, ParameterType.IQ_FILTERED)
                torque = bus.read(MOTOR_NAME, ParameterType.MEASURED_TORQUE)
                print(
                    f"   v={velocity:+.3f} rad/s  "
                    f"iq={iq_fdb:+.3f} A  tau={torque:+.3f} Nm"
                )
                time.sleep(SAMPLE_PERIOD_SECONDS)

    finally:
        # Wind the current down to zero BEFORE disabling so the motor does
        # not jerk on shutdown.
        try:
            bus.set_target_current(MOTOR_NAME, iq=0.0)
            time.sleep(0.2)
        finally:
            try:
                bus.disable(MOTOR_NAME)
            finally:
                bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
