"""
Operation Control (MIT) Mode Example.
运控模式(MIT 模式)示例

This example shows how to drive a Robstride motor in operation control mode
(also known as MIT mode). In this mode, every control frame carries the target
position, feed-forward velocity, feed-forward torque, and the impedance gains
(kp, kd). The motor responds to each command with a status frame containing
the measured position, velocity, torque, and temperature.

本示例演示如何通过运控模式(MIT 模式)驱动 Robstride 电机：每一帧控制指令包
括目标位置、前馈速度、前馈力矩以及阻抗参数 kp/kd; 电机在每帧之后会回复一个
状态帧，其中包含位置、速度、力矩与温度反馈。

Before running:
    - Bring up the SocketCAN interface, e.g.
        $ sudo ip link set can0 up type can bitrate 1000000
    - Connect the motor's CAN bus and power.
    - Make sure the motor's id and model below match your hardware.

CAUTION:
    The motor will move when this script runs. Keep clear of the output shaft.
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
"""Model string used to look up the MIT scaling tables in `table.py`."""

CONTROL_FREQUENCY_HZ = 200.0
"""Control loop frequency. 200 Hz is a reasonable default for most use cases."""

DURATION_SECONDS = 6.0
"""How long to run the trajectory."""

TRAJECTORY_AMPLITUDE_RAD = 0.5
"""Amplitude of the sinusoidal position reference, in rad."""

TRAJECTORY_FREQUENCY_HZ = 0.5
"""Frequency of the sinusoidal position reference, in Hz."""

KP = 20.0
"""Proportional gain (Nm/rad). Tune for your motor and load."""

KD = 1.0
"""Derivative gain (Nm/(rad/s)). Tune for your motor and load."""


def main() -> None:
    bus = RobstrideBus(
        channel=CHANNEL,
        motors={MOTOR_NAME: Motor(id=MOTOR_ID, model=MOTOR_MODEL)},
    )
    bus.connect()

    try:
        # Switch the motor to operation control mode (run_mode = 0).
        # 切换到运控模式 (run_mode = 0)
        bus.write(MOTOR_NAME, ParameterType.MODE, 0)

        # Enable torque output. The motor will hold its current position
        # until we start sending operation frames.
        bus.enable(MOTOR_NAME)

        # Read the current position and use it as the trajectory's center,
        # so the motor does not jump when the loop starts.
        bus.write_operation_frame(MOTOR_NAME, position=0.0, kp=0.0, kd=0.0, velocity=0.0, torque=0.0)
        start_position, _, _, _ = bus.read_operation_frame(MOTOR_NAME)
        print(f"Starting from position: {start_position:.4f} rad")

        period = 1.0 / CONTROL_FREQUENCY_HZ
        loop_start = time.perf_counter()
        next_tick = loop_start

        while True:
            elapsed = time.perf_counter() - loop_start
            if elapsed >= DURATION_SECONDS:
                break

            target_position = start_position + TRAJECTORY_AMPLITUDE_RAD * math.sin(
                2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ * elapsed
            )
            target_velocity = (
                TRAJECTORY_AMPLITUDE_RAD
                * 2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ
                * math.cos(2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ * elapsed)
            )

            bus.write_operation_frame(
                motor=MOTOR_NAME,
                position=target_position,
                kp=KP,
                kd=KD,
                velocity=target_velocity,
                torque=0.0,
            )

            position, velocity, torque, temperature = bus.read_operation_frame(MOTOR_NAME)
            print(
                f"t={elapsed:5.2f}s  "
                f"pos_ref={target_position:+.3f}  pos={position:+.3f} rad  "
                f"vel={velocity:+.3f} rad/s  tau={torque:+.3f} Nm  "
                f"T={temperature:5.1f} C"
            )

            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # We fell behind the control rate; resync the schedule.
                next_tick = time.perf_counter()

    finally:
        # Disabling torque is also done by `disconnect`, but we make it
        # explicit here so the motor stops immediately even if the
        # `disconnect` call fails.
        try:
            bus.disable(MOTOR_NAME)
        finally:
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
