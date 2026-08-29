"""
NEUTRAL 자세를 거쳐 목표 위치로 이동시키는 스크립트.
모터를 1번부터 순서대로 "하나씩" 이동시키는 방식 (동시이동 아님).

PID 클로즈드루프 보정이 제거되어 목표 위치로 모터를 순서대로 이동시킨 뒤 종료합니다.
"""

import time
from scservo_sdk import *

DEVICENAME_LEFT = "COM12"
DEVICENAME_RIGHT = "COM14"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
TORQUE_ENABLE = 1

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]

# =====================================================================
# ===== 실측 스테이션 목표값 =====
# =====================================================================
STATION_RAW_TARGETS = {
    ("right", "fine"): [2386, 1195, 2707, 1607, 3446, 1757, 0],
    ("right", "nipper"): [2536, 1204, 2686, 1763, 3450, 1654, 0],
    ("left", "default"): [1406, 1400, 1855, 1790, 245, 1722, 0],
    ("left", "vise"): [1234, 1348, 1850, 1735, 258, 1774, 0],
}

BACKLASH_OFFSET = {
    ("right", "fine"): [17, 0, -5, -13, 4, -2, 4],
    ("right", "nipper"): [21, 3, 3, -15, 4, -2, -2],
    ("left", "default"): [-16, 7, -4, -9, -2, -3, -3],
    ("left", "vise"): [0, -7, -4, -10, 3, -6, -3],
}


def get_corrected_target(arm_side, station_name):
    """실측 목표값에 백래시 보정을 적용해서 반환."""
    raw = STATION_RAW_TARGETS[(arm_side, station_name)]
    offset = BACKLASH_OFFSET.get((arm_side, station_name))
    if offset is None:
        return list(raw)
    return [int(t - o) for t, o in zip(raw, offset)]


# ===== 테스트 대상 스테이션 설정 =====
TARGET_STATION_LEFT = ("left", "vise")
TARGET_STATION_RIGHT = ("right", "nipper")

TARGET_TICKS_LEFT = get_corrected_target(*TARGET_STATION_LEFT)
TARGET_TICKS_RIGHT = get_corrected_target(*TARGET_STATION_RIGHT)

# ===== 파라미터 설정 =====
NEUTRAL_WAIT_SECONDS = 2.0
NEUTRAL_TRANSITION_SECONDS = 2.0
NEUTRAL_TRANSITION_STEPS = 40

MOTOR_BY_MOTOR_DURATION = 1.0
MOTOR_BY_MOTOR_STEPS = 10


def move_arm(portHandler, packetHandler, motors, ticks):
    for m, tick in zip(motors, ticks):
        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, int(tick))


def read_current_ticks(portHandler, packetHandler, motors):
    ticks = []
    for m in motors:
        pos, result, error = packetHandler.read2ByteTxRx(
            portHandler, m, ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            print(
                f"    [경고] 모터{m}번 현재위치 읽기 실패: {packetHandler.getTxRxResult(result)}"
            )
            ticks.append(None)
        else:
            ticks.append(pos)
    return ticks


def read_current_ticks_safe(portHandler, packetHandler, motors, fallback_ticks):
    raw_ticks = read_current_ticks(portHandler, packetHandler, motors)
    safe_ticks = []
    for idx, t in enumerate(raw_ticks):
        if t is None:
            safe_ticks.append(fallback_ticks[idx])
        else:
            safe_ticks.append(t)
    return safe_ticks


def _lerp_ticks(start_ticks, end_ticks, ratio):
    result = []
    for s, e in zip(start_ticks, end_ticks):
        if s is None or e is None:
            result.append(e)
        else:
            result.append(int(round(s + (e - s) * ratio)))
    return result


def move_arm_gradually(
    portHandler, packetHandler, motors, from_ticks, to_ticks, duration_seconds, steps
):
    step_delay = duration_seconds / steps
    for step in range(1, steps + 1):
        ratio = step / steps
        interp = _lerp_ticks(from_ticks, to_ticks, ratio)
        move_arm(portHandler, packetHandler, motors, interp)
        time.sleep(step_delay)


def move_arm_motor_by_motor(
    portHandler,
    packetHandler,
    motors,
    from_ticks,
    to_ticks,
    duration_per_motor=MOTOR_BY_MOTOR_DURATION,
    steps_per_motor=MOTOR_BY_MOTOR_STEPS,
):
    working = list(from_ticks)
    for idx in range(len(working)):
        if to_ticks[idx] is None or working[idx] is None:
            continue
        start_tick = working[idx]
        target_tick = to_ticks[idx]
        if start_tick == target_tick:
            print(f"    모터{motors[idx]}번: 이미 목표값과 같음 — 건너뜀")
            continue

        print(f"    모터{motors[idx]}번 이동 시작: {start_tick} → {target_tick}")
        step_delay = duration_per_motor / steps_per_motor
        for step in range(1, steps_per_motor + 1):
            ratio = step / steps_per_motor
            working[idx] = int(round(start_tick + (target_tick - start_tick) * ratio))
            move_arm(portHandler, packetHandler, motors, working)
            time.sleep(step_delay)
        working[idx] = target_tick
        print(f"    모터{motors[idx]}번 도착")
    return working


def main():
    print(
        f">>> 목표 스테이션: 왼팔={TARGET_STATION_LEFT}, 오른팔={TARGET_STATION_RIGHT}"
    )
    print(f"    보정 적용된 왼팔 목표: {TARGET_TICKS_LEFT}")
    print(f"    보정 적용된 오른팔 목표: {TARGET_TICKS_RIGHT}")

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

    for m in MOTORS_LEFT:
        packetHandler_left.write1ByteTxRx(
            portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
    for m in MOTORS_RIGHT:
        packetHandler_right.write1ByteTxRx(
            portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )

    # ===== 0단계: 실제 현재 위치 읽기 =====
    print(">>> 0단계: 서보 현재 위치 읽는 중...")
    current_left = read_current_ticks_safe(
        portHandler_left, packetHandler_left, MOTORS_LEFT, NEUTRAL_TICKS_LEFT
    )
    current_right = read_current_ticks_safe(
        portHandler_right, packetHandler_right, MOTORS_RIGHT, NEUTRAL_TICKS_RIGHT
    )

    print(f"    왼팔 시작 위치: {current_left}")
    print(f"    오른팔 시작 위치: {current_right}")

    # ----- 1단계: NEUTRAL 자세로 이동 -----
    print(">>> 1단계: NEUTRAL 자세로 한 번에 이동 (왼팔)")
    move_arm_gradually(
        portHandler_left,
        packetHandler_left,
        MOTORS_LEFT,
        current_left,
        NEUTRAL_TICKS_LEFT,
        NEUTRAL_TRANSITION_SECONDS,
        NEUTRAL_TRANSITION_STEPS,
    )
    print(">>> 1단계: NEUTRAL 자세로 한 번에 이동 (오른팔)")
    move_arm_gradually(
        portHandler_right,
        packetHandler_right,
        MOTORS_RIGHT,
        current_right,
        NEUTRAL_TICKS_RIGHT,
        NEUTRAL_TRANSITION_SECONDS,
        NEUTRAL_TRANSITION_STEPS,
    )

    # ----- 2단계: 대기 -----
    print(f">>> 2단계: {NEUTRAL_WAIT_SECONDS}초 대기 중...")
    time.sleep(NEUTRAL_WAIT_SECONDS)

    # ----- 3단계: 목표 위치로 하나씩 순서대로 이동 -----
    print(">>> 3단계: 목표 위치로 모터 하나씩 순서대로 이동 (왼팔)")
    move_arm_motor_by_motor(
        portHandler_left,
        packetHandler_left,
        MOTORS_LEFT,
        NEUTRAL_TICKS_LEFT,
        TARGET_TICKS_LEFT,
    )

    print(">>> 3단계: 목표 위치로 모터 하나씩 순서대로 이동 (오른팔)")
    move_arm_motor_by_motor(
        portHandler_right,
        packetHandler_right,
        MOTORS_RIGHT,
        NEUTRAL_TICKS_RIGHT,
        TARGET_TICKS_RIGHT,
    )

    print(">>> 완료")

    portHandler_left.closePort()
    portHandler_right.closePort()


if __name__ == "__main__":
    main()
