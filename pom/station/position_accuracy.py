"""
"명령한 tick vs 실제 도달한 tick" 오차를 측정하는 진단 스크립트.

두 가지 테스트:
  [테스트 1] 스테이션 정확도 — 목표 위치로 이동시키고, 명령값과 실제
             present position이 얼마나 차이나는지 관절별로 확인
  [테스트 2] 그리퍼 반작용 — 그리퍼(7번 모터)를 조일 때, 다른 관절
             (1~6번)의 present position이 같이 흔들리는지 실시간 기록

결과는 콘솔에 표로 출력하고, position_diagnostic_log.csv 파일로도 저장됨
(나중에 엑셀 등으로 그래프 그려서 패턴 분석 가능).
"""

import time
import csv
from scservo_sdk import *

DEVICENAME_LEFT = "COM12"
DEVICENAME_RIGHT = "COM14"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56  # ⚠️ 추정값

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]

CSV_LOG_PATH = "position_diagnostic_log.csv"


def open_port(device_name):
    port_handler = PortHandler(device_name)
    packet_handler = PacketHandler(PROTOCOL_END)
    if not port_handler.openPort():
        raise RuntimeError(f"{device_name} 포트 열기 실패")
    if not port_handler.setBaudRate(BAUDRATE):
        raise RuntimeError(f"{device_name} 보드레이트 설정 실패")
    return port_handler, packet_handler


def enable_torque(port_handler, packet_handler, motors):
    for m in motors:
        packet_handler.write1ByteTxRx(port_handler, m, ADDR_TORQUE_ENABLE, 1)


def write_goal(port_handler, packet_handler, motor_id, tick):
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_GOAL_POSITION, int(tick))


def read_present(port_handler, packet_handler, motor_id):
    pos, result, error = packet_handler.read2ByteTxRx(
        port_handler, motor_id, ADDR_PRESENT_POSITION
    )
    if result != COMM_SUCCESS:
        return None
    return pos


def write_all(port_handler, packet_handler, motors, ticks):
    for m, t in zip(motors, ticks):
        if t is not None:
            write_goal(port_handler, packet_handler, m, t)


# ===== 모터별 순차이동 파라미터 (목표/스테이션 tick 이동에 사용) =====
MOTOR_BY_MOTOR_DURATION = 0.3  # 모터 하나 이동에 걸리는 시간(초)
MOTOR_BY_MOTOR_STEPS = 10  # 모터 하나 이동을 몇 단계로 나눌지


def move_arm_motor_by_motor(
    port_handler,
    packet_handler,
    motors,
    from_ticks,
    to_ticks,
    duration_per_motor=MOTOR_BY_MOTOR_DURATION,
    steps_per_motor=MOTOR_BY_MOTOR_STEPS,
):
    """
    from_ticks에서 to_ticks까지, 모터를 1번부터 순서대로 하나씩 이동.
    (실제 그리퍼교체 로직과 똑같은 방식 — 한 모터가 목표에 도달해야 다음 모터로 넘어감)
    이미 목표와 같은 모터는 건너뜀.
    """
    working = list(from_ticks)
    for idx in range(len(working)):
        if to_ticks[idx] is None or working[idx] is None:
            continue
        start_tick = working[idx]
        target_tick = to_ticks[idx]
        if start_tick == target_tick:
            continue
        step_delay = duration_per_motor / steps_per_motor
        for step in range(1, steps_per_motor + 1):
            ratio = step / steps_per_motor
            working[idx] = int(round(start_tick + (target_tick - start_tick) * ratio))
            write_all(port_handler, packet_handler, motors, working)
            time.sleep(step_delay)
        working[idx] = target_tick
    return working


# ===== NEUTRAL 전용: 7개 모터 동시(한 번에) 이동 =====
NEUTRAL_TRANSITION_SECONDS = 2.0  # 정자세로 이동하는데 걸리는 시간
NEUTRAL_TRANSITION_STEPS = 40  # 몇 단계로 나눠서 보간할지


def _lerp_ticks(start_ticks, end_ticks, ratio):
    result = []
    for s, e in zip(start_ticks, end_ticks):
        if s is None or e is None:
            result.append(e)
        else:
            result.append(int(round(s + (e - s) * ratio)))
    return result


def move_arm_gradually(
    port_handler,
    packet_handler,
    motors,
    from_ticks,
    to_ticks,
    duration_seconds=NEUTRAL_TRANSITION_SECONDS,
    steps=NEUTRAL_TRANSITION_STEPS,
):
    """
    (NEUTRAL 전용) from_ticks에서 to_ticks까지 7개 모터 전부 동시에 보간 이동.
    """
    step_delay = duration_seconds / steps
    for step in range(1, steps + 1):
        ratio = step / steps
        interp = _lerp_ticks(from_ticks, to_ticks, ratio)
        write_all(port_handler, packet_handler, motors, interp)
        time.sleep(step_delay)
    return list(to_ticks)


def read_all(port_handler, packet_handler, motors):
    return [read_present(port_handler, packet_handler, m) for m in motors]


def print_diff_table(label, motors, commanded, actual):
    print(f"\n[{label}] 명령값 vs 실제값")
    print(f"{'모터ID':>6} {'명령값':>8} {'실제값':>8} {'오차':>8}")
    for m, c, a in zip(motors, commanded, actual):
        if a is None:
            print(f"{m:>6} {c:>8} {'읽기실패':>8} {'-':>8}")
        else:
            diff = a - c
            print(f"{m:>6} {c:>8} {a:>8} {diff:>+8}")


def test_station_accuracy(arm_side):
    """목표 위치로 이동시키고 명령값 vs 실제값 비교."""
    device = DEVICENAME_LEFT if arm_side == "left" else DEVICENAME_RIGHT
    motors = MOTORS_LEFT if arm_side == "left" else MOTORS_RIGHT
    neutral = NEUTRAL_TICKS_LEFT if arm_side == "left" else NEUTRAL_TICKS_RIGHT

    print(f"\n{'='*60}")
    print(f"[테스트 1] {arm_side} 팔 — 스테이션 정확도 테스트")
    print(f"{'='*60}")

    target_str = input(
        f"목표 tick 7개를 콤마로 입력하세요 (예: 1424,1139,1275,1580,1790,2102,2631)\n> "
    )
    target = [int(x.strip()) for x in target_str.split(",")]
    if len(target) != 7:
        print("7개가 아닙니다. 취소합니다.")
        return

    port_handler, packet_handler = open_port(device)
    enable_torque(port_handler, packet_handler, motors)

    print(">>> 현재 위치 읽는 중...")
    current = read_all(port_handler, packet_handler, motors)
    if not all(t is not None for t in current):
        print(
            "    [경고] 일부 모터 위치 읽기 실패 — NEUTRAL로 대체(첫 이동이 급격할 수 있음)"
        )
        current = neutral

    print(">>> NEUTRAL로 한 번에(동시에) 이동...")
    current = move_arm_gradually(port_handler, packet_handler, motors, current, neutral)
    time.sleep(2.0)

    print(">>> 목표 위치로 모터 하나씩 순서대로 이동...")
    move_arm_motor_by_motor(port_handler, packet_handler, motors, current, target)
    time.sleep(1.0)  # 도착 안정화

    actual = read_all(port_handler, packet_handler, motors)
    print_diff_table(f"{arm_side} 스테이션 정확도", motors, target, actual)

    with open(CSV_LOG_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"station_accuracy_{arm_side}", time.strftime("%H:%M:%S")])
        writer.writerow(["motor_id", "commanded", "actual", "diff"])
        for m, c, a in zip(motors, target, actual):
            writer.writerow([m, c, a, (a - c) if a is not None else "N/A"])
        writer.writerow([])

    port_handler.closePort()
    print(f"\n>>> 결과가 {CSV_LOG_PATH}에도 저장되었습니다.")


def test_gripper_effect(arm_side):
    """그리퍼(7번 모터)를 조일 때 다른 관절이 같이 흔들리는지 실시간 기록."""
    device = DEVICENAME_LEFT if arm_side == "left" else DEVICENAME_RIGHT
    motors = MOTORS_LEFT if arm_side == "left" else MOTORS_RIGHT

    print(f"\n{'='*60}")
    print(f"[테스트 2] {arm_side} 팔 — 그리퍼 반작용 테스트")
    print(f"{'='*60}")

    station_str = input(
        "현재 스테이션 위치 tick 7개를 콤마로 입력하세요 (그리퍼 자리)\n> "
    )
    station_ticks = [int(x.strip()) for x in station_str.split(",")]
    if len(station_ticks) != 7:
        print("7개가 아닙니다. 취소합니다.")
        return

    close_tick = int(input("그리퍼 MAX_CLOSE 값 입력\n> "))
    open_tick = int(input("그리퍼 MAX_OPEN 값 입력\n> "))
    duration = float(input("몇 초 동안 기록할지 (예: 5)\n> ") or "5")
    interval = 0.2

    port_handler, packet_handler = open_port(device)
    enable_torque(port_handler, packet_handler, motors)

    print(">>> 현재 위치 읽는 중...")
    current = read_all(port_handler, packet_handler, motors)
    if not all(t is not None for t in current):
        print(
            "    [경고] 일부 모터 위치 읽기 실패 — station_ticks 자리로 대체(첫 이동이 급격할 수 있음)"
        )
        current = station_ticks

    print(">>> 스테이션 위치로 모터 하나씩 순서대로 이동...")
    move_arm_motor_by_motor(
        port_handler, packet_handler, motors, current, station_ticks
    )
    time.sleep(1.0)  # 도착 안정화

    print(">>> 그리퍼 조이기 전 — 기준값(baseline) 읽기")
    baseline = read_all(port_handler, packet_handler, motors)
    print_diff_table("조이기 전(baseline)", motors, station_ticks, baseline)

    print(
        f"\n>>> 그리퍼를 MAX_CLOSE({close_tick})로 조입니다. {duration}초간 전체 관절 위치 기록..."
    )
    write_goal(port_handler, packet_handler, motors[6], close_tick)

    log_rows = []
    start = time.time()
    while time.time() - start < duration:
        t = time.time() - start
        positions = read_all(port_handler, packet_handler, motors)
        log_rows.append([round(t, 2)] + positions)
        drift_str = " ".join(
            f"{m}:{(p - b):+d}" if p is not None and b is not None else f"{m}:?"
            for m, p, b in zip(motors[:6], positions[:6], baseline[:6])
        )
        print(
            f"  t={t:5.2f}s  1~6번 관절 baseline 대비 오차: {drift_str}   그리퍼(7번):{positions[6]}"
        )
        time.sleep(interval)

    print("\n>>> 그리퍼를 다시 MAX_OPEN으로 펴는 중...")
    write_goal(port_handler, packet_handler, motors[6], open_tick)
    time.sleep(2.0)

    final = read_all(port_handler, packet_handler, motors)
    print_diff_table("그리퍼 개방 후", motors, station_ticks, final)

    with open(CSV_LOG_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"gripper_effect_{arm_side}", time.strftime("%H:%M:%S")])
        writer.writerow(["time_sec"] + [f"motor_{m}" for m in motors])
        for row in log_rows:
            writer.writerow(row)
        writer.writerow([])

    port_handler.closePort()
    print(
        f"\n>>> 시계열 데이터가 {CSV_LOG_PATH}에 저장되었습니다 (엑셀에서 그래프 그려보세요)."
    )


def main():
    print("=" * 60)
    print("서보 위치 정확도 진단 도구")
    print("=" * 60)
    print("1) 테스트 1: 스테이션 정확도 (명령값 vs 실제값)")
    print("2) 테스트 2: 그리퍼 반작용 (조일 때 다른 관절 흔들림 확인)")
    choice = input("선택 (1 또는 2): ").strip()

    arm_side = input("어느 팔? (left / right): ").strip().lower()
    if arm_side not in ("left", "right"):
        print("left 또는 right만 입력하세요.")
        return

    if choice == "1":
        test_station_accuracy(arm_side)
    elif choice == "2":
        test_gripper_effect(arm_side)
    else:
        print("1 또는 2만 입력하세요.")


if __name__ == "__main__":
    main()
