from scservo_sdk import *

DEVICENAME_LEFT = "COM12"
DEVICENAME_RIGHT = "COM14"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
TORQUE_ENABLE = 1

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

TARGET_TICKS_LEFT = [
    1457,
    1145,
    1275,
    1580,
    1790,
    2102,
    2725,
]  # 왼팔 기본(디폴트)그리퍼 실측값
TARGET_TICKS_RIGHT = [
    2549,
    990,
    3078,
    1600,
    2020,
    2237,
    2658,
]  # 오른팔 미세그리퍼 실측값


def move_arm(portHandler, packetHandler, motors, ticks):
    for m in motors:
        packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    for m, tick in zip(motors, ticks):
        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)


def main():
    portHandler_left = PortHandler(DEVICENAME_LEFT)
    packetHandler_left = PacketHandler(PROTOCOL_END)
    if not portHandler_left.openPort():
        print(f"왼팔 포트({DEVICENAME_LEFT}) 열기 실패")
        return
    portHandler_left.setBaudRate(BAUDRATE)

    portHandler_right = PortHandler(DEVICENAME_RIGHT)
    packetHandler_right = PacketHandler(PROTOCOL_END)
    if not portHandler_right.openPort():
        print(f"오른팔 포트({DEVICENAME_RIGHT}) 열기 실패")
        return
    portHandler_right.setBaudRate(BAUDRATE)

    move_arm(portHandler_left, packetHandler_left, MOTORS_LEFT, TARGET_TICKS_LEFT)
    print(">>> 왼팔 이동 명령 전송 완료:", TARGET_TICKS_LEFT)

    move_arm(portHandler_right, packetHandler_right, MOTORS_RIGHT, TARGET_TICKS_RIGHT)
    print(">>> 오른팔 이동 명령 전송 완료:", TARGET_TICKS_RIGHT)

    portHandler_left.closePort()
    portHandler_right.closePort()


if __name__ == "__main__":
    main()
