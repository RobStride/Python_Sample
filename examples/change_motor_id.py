"""
Change Motor CAN ID Example.
修改电机 CAN ID 示例

This example changes a Robstride motor's CAN ID through the private protocol.
Only connect one motor, or make sure there is no other motor using either the
old ID or the new ID on the same CAN bus.

本示例通过私有协议修改 Robstride 电机的 CAN ID。运行前请确保总线上只有目标
电机，或至少确认旧 ID / 新 ID 不会与其它电机冲突。

Before running:
    - Bring up the SocketCAN interface, e.g.
        $ sudo ip link set can0 up type can bitrate 1000000
    - Connect the motor's CAN bus and power.
    - Edit OLD_MOTOR_ID and NEW_MOTOR_ID below.

CAUTION:
    Changing CAN ID is a persistent device configuration. Double-check the IDs
    before running this script.
"""

from robstride_dynamics import Motor, RobstrideBus


CHANNEL = "can1"
"""SocketCAN channel that the motor is connected to."""

MOTOR_NAME = "joint_1"
"""Friendly name used to refer to the motor in the bus."""

# 先用candump canX 查看原来电机是哪个id，电机上电会自动返回两帧，上面有电机id，帧头9-16位是id
OLD_MOTOR_ID = 0x01
"""Current CAN id of the motor."""

# 修改为新的id
NEW_MOTOR_ID = 0x02
"""New CAN id to write to the motor."""

MOTOR_MODEL = "rs-01"
"""Model string. ID writing does not use scaling tables, but Motor requires it."""


def main() -> None:
    if not 1 <= OLD_MOTOR_ID <= 0xFF:
        raise ValueError(f"OLD_MOTOR_ID out of range: {OLD_MOTOR_ID}")
    if not 1 <= NEW_MOTOR_ID <= 0xFF:
        raise ValueError(f"NEW_MOTOR_ID out of range: {NEW_MOTOR_ID}")
    if OLD_MOTOR_ID == NEW_MOTOR_ID:
        raise ValueError("OLD_MOTOR_ID and NEW_MOTOR_ID are the same")

    bus = RobstrideBus(
        channel=CHANNEL,
        motors={MOTOR_NAME: Motor(id=OLD_MOTOR_ID, model=MOTOR_MODEL)},
    )
    bus.connect(handshake=False)

    try:
        print(f"Checking motor at old ID: 0x{OLD_MOTOR_ID:02X}")
        old_response = bus.read_id(MOTOR_NAME, timeout=0.5)
        if old_response is None:
            raise RuntimeError(
                f"No motor responded at old ID 0x{OLD_MOTOR_ID:02X}. "
                "Check wiring, bitrate, and OLD_MOTOR_ID."
            )

        print(f"Writing new ID: 0x{NEW_MOTOR_ID:02X}")
        result = bus.write_id(MOTOR_NAME, NEW_MOTOR_ID)
        if result is None:
            raise RuntimeError("Motor did not acknowledge the ID change command.")

        print(f"Verifying motor at new ID: 0x{NEW_MOTOR_ID:02X}")
        new_response = bus.read_id(MOTOR_NAME, timeout=0.5)
        if new_response is None:
            bus.motors[MOTOR_NAME].id = OLD_MOTOR_ID
            old_still_online = bus.read_id(MOTOR_NAME, timeout=0.5)
            if old_still_online is not None:
                raise RuntimeError(
                    f"ID write returned, but the motor still responds at old ID "
                    f"0x{OLD_MOTOR_ID:02X}. The ID change did not take effect."
                )
            raise RuntimeError(
                f"ID write returned, but no motor responded at new ID 0x{NEW_MOTOR_ID:02X} "
                f"or old ID 0x{OLD_MOTOR_ID:02X}. Power-cycle the motor and scan the bus."
            )

        print(
            "ID change complete: "
            f"0x{OLD_MOTOR_ID:02X} -> 0x{NEW_MOTOR_ID:02X}"
        )
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
