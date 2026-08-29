"""
NEUTRAL 자세를 거쳐 목표 위치로 이동시키는 스크립트.
모터를 1번부터 순서대로 "하나씩" 이동시키는 방식 (동시이동 아님).

3단계(목표 위치 이동)는 closed_loop_correction.py를 이용해서
"이동 → 실제 위치 확인 → 오차만큼 재조정 → 재확인"을 자동으로 반복함.
백래시/유격 오차가 매번 다르게 나더라도, 허용오차 이내로 수렴할 때까지
스스로 보정하기 때문에 고정 보정표(BACKLASH_OFFSET)보다 더 견고함.
"""

import time
from scservo_sdk import *

import closed_loop_correction as clc

DEVICENAME_LEFT = "COM12"
DEVICENAME_RIGHT = "COM14"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56  # ⚠️ 추정값 — 실기 검증 필요
TORQUE_ENABLE = 1

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]  # 모터 1~7
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]  # 모터 9~15

# =====================================================================
# ===== 실측 스테이션 목표값 (2026-08-09 position_accuracy_diagnostic.py로 측정) =====
# =====================================================================
STATION_RAW_TARGETS = {
    ("right", "fine"): [2540, 1017, 3078, 1586, 2075, 2183, 2549],
    ("right", "nipper"): [2495, 845, 3113, 1736, 1917, 2264, 1718],
    ("left", "default"): [1437, 1010, 1276, 1593, 1864, 2145, 2484],
    ("left", "vise"): [1437, 1153, 1275, 1580, 1790, 2183, 2156],
}

# ===== 백래시 보정값 (station_positions.py와 동일한 값) =====
# ⚠️ 참고용으로 남겨둠 — 클로즈드루프가 실시간으로 알아서 맞춰주기 때문에
#    지금은 "시작점(첫 시도값)"으로만 쓰이고, 필수는 아님. 클로즈드루프 없이
#    쓰고 싶으면 이 오프셋을 그대로 최종 명령값으로 써도 됨.
# 오차 = 실제값 - 명령값 (관측됨). 보정 명령값 = 원래 목표값 - 오차
BACKLASH_OFFSET = {
    ("right", "fine"): [17, 0, -5, -13, 4, -2, 4],
    ("right", "nipper"): [11, 3, 3, -15, 4, -2, -2],
    ("left", "default"): [-16, 7, -4, -9, -2, -3, -3],
    ("left", "vise"): [-18, -7, -4, -10, 3, -6, -3],
}


def get_corrected_target(arm_side, station_name):
    """실측 목표값에 백래시 보정을 적용해서 반환 (클로즈드루프의 첫 시도값으로 사용)."""
    raw = STATION_RAW_TARGETS[(arm_side, station_name)]
    offset = BACKLASH_OFFSET.get((arm_side, station_name))
    if offset is None:
        return list(raw)
    return [int(t - o) for t, o in zip(raw, offset)]


# ===== 여기서 테스트할 스테이션을 고르세요 =====
# 선택지: ("right","fine") / ("right","nipper") / ("left","default") / ("left","vise")
TARGET_STATION_LEFT = ("left", "default")
TARGET_STATION_RIGHT = ("right", "nipper")

TARGET_TICKS_LEFT = get_corrected_target(*TARGET_STATION_LEFT)
TARGET_TICKS_RIGHT = get_corrected_target(*TARGET_STATION_RIGHT)

# ===== 대기 시간 =====
NEUTRAL_WAIT_SECONDS = 2.0  # 정자세에서 대기하는 시간 (흔들림 방지)

NEUTRAL_TRANSITION_SECONDS = (
    2.0  # 정자세로 "천천히" 이동하는데 걸리는 시간(동시이동, 1번만 사용)
)
NEUTRAL_TRANSITION_STEPS = 40  # 몇 단계로 나눠서 보간할지

# ===== 모터별 순차이동 파라미터 (정자세 이후 목표위치 이동에서 사용) =====
MOTOR_BY_MOTOR_DURATION = 1.0  # 모터 하나 이동에 걸리는 시간(초)
MOTOR_BY_MOTOR_STEPS = 10  # 모터 하나 이동을 몇 단계로 나눌지 (많을수록 부드러움)

# ===== 클로즈드루프 파라미터 =====
CORRECTION_TOLERANCE = 5  # 이 오차(tick) 이내면 "도달 성공"으로 판정
CORRECTION_MAX_RETRIES = 3  # 최대 재시도 횟수
CORRECTION_SETTLE_DELAY = 0.3  # 재확인 전 대기시간(초)


def move_arm(portHandler, packetHandler, motors, ticks):
    """한 번에 목표 tick으로 write (보간의 한 스텝, 또는 단발성 이동에 사용)."""
    for m, tick in zip(motors, ticks):
        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, int(tick))


def read_current_ticks(portHandler, packetHandler, motors):
    """실제 서보 현재 위치 읽기. 읽기 실패한 모터는 None으로 채움."""
    ticks = []
    for m in motors:
        pos, result, error = packetHandler.read2ByteTxRx(
            portHandler, m, ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            print(
                f"  [경고] 모터{m}번 현재위치 읽기 실패: {packetHandler.getTxRxResult(result)}"
            )
            ticks.append(None)
        else:
            ticks.append(pos)
    return ticks


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
    """
    (정자세 전용) from_ticks에서 to_ticks까지 7개 모터 전부 동시에 보간 이동.
    여러 번 나눠 움직이지 않고 한 번에(동시에) 움직이는 방식.
    """
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
    """
    from_ticks에서 to_ticks까지, 모터를 1번부터 순서대로 하나씩 이동.
    (7개 동시가 아니라, 1번 모터가 목표에 다 도달해야 2번 모터가 움직이기 시작)
    각 모터 자체의 이동은 보간으로 부드럽게 처리. 이미 목표와 같은 모터는 건너뜀.
    """
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


def move_to_target_with_correction(
    portHandler, packetHandler, motors, from_ticks, target_ticks, arm_label
):
    """
    목표 위치로 모터별 순차이동 → 클로즈드루프로 실제 도달값 확인 및 재보정.
    closed_loop_correction.move_with_correction()에 이 파일의 move_fn/read_fn을
    감싸서 넘겨주는 역할.
    """
    print(f">>> 3단계: 목표 위치로 모터 하나씩 순서대로 이동 ({arm_label})")
    move_arm_motor_by_motor(
        portHandler, packetHandler, motors, from_ticks, target_ticks
    )

    print(f">>> 3-1단계: 클로즈드루프 보정 시작 ({arm_label})")

    def move_fn(ticks):
        move_arm(portHandler, packetHandler, motors, ticks)

    def read_fn():
        return read_current_ticks(portHandler, packetHandler, motors)

    final_ticks, success = clc.move_with_correction(
        target_ticks=target_ticks,
        move_fn=move_fn,
        read_fn=read_fn,
        tolerance=CORRECTION_TOLERANCE,
        max_retries=CORRECTION_MAX_RETRIES,
        settle_delay=CORRECTION_SETTLE_DELAY,
    )

    status = "성공" if success else "실패(마지막 값 유지)"
    print(
        f">>> 3-1단계: 클로즈드루프 보정 {status} ({arm_label}) — 최종: {final_ticks}\n"
    )
    return final_ticks


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

    # ===== 0단계: 실제 현재 위치 읽기 (여기서부터 시작해야 첫 이동이 안 튐) =====
    print(">>> 0단계: 서보 현재 위치 읽는 중...")
    current_left = read_current_ticks(portHandler_left, packetHandler_left, MOTORS_LEFT)
    current_right = read_current_ticks(
        portHandler_right, packetHandler_right, MOTORS_RIGHT
    )

    if not all(t is not None for t in current_left):
        print(
            "    [경고] 왼팔 위치 읽기 일부 실패 — NEUTRAL로 대체(첫 이동이 급격할 수 있음)"
        )
        current_left = NEUTRAL_TICKS_LEFT
    else:
        print(f"    왼팔 현재 위치: {current_left}")

    if not all(t is not None for t in current_right):
        print(
            "    [경고] 오른팔 위치 읽기 일부 실패 — NEUTRAL로 대체(첫 이동이 급격할 수 있음)"
        )
        current_right = NEUTRAL_TICKS_RIGHT
    else:
        print(f"    오른팔 현재 위치: {current_right}")

    # ----- 1단계: NEUTRAL(일자로 편 자세)로 한 번에(동시에) 이동 -----
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

    # ----- 3단계: 목표 위치로 모터 하나씩 순서대로 이동 + 클로즈드루프 보정 -----
    move_to_target_with_correction(
        portHandler_left,
        packetHandler_left,
        MOTORS_LEFT,
        NEUTRAL_TICKS_LEFT,
        TARGET_TICKS_LEFT,
        "왼팔",
    )
    move_to_target_with_correction(
        portHandler_right,
        packetHandler_right,
        MOTORS_RIGHT,
        NEUTRAL_TICKS_RIGHT,
        TARGET_TICKS_RIGHT,
        "오른팔",
    )

    print(">>> 완료")

    portHandler_left.closePort()
    portHandler_right.closePort()


if __name__ == "__main__":
    main()
