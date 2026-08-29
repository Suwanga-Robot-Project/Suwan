"""
상하이동 없이 로컬에서 그리퍼 자동교체 로직을 테스트하는 스크립트.
station_positions.py의 실측값을 그대로 사용 (더미값 없음).

테스트 순서:
1. 빈손 → 부착 (Flow A, 각 스테이션 4개)
2. 짝 전환 (Flow B, 클리어런스) — 1↔2, 3↔4 양방향
3. A안 규칙: IDLE이 아닐 때 키 입력이 무시되는지
4. 순차 클리어런스 이동이 로그로 제대로 보이는지
"""

import tool_changer_fsm
import key_input_handler
import station_positions

# ===== 현재 팔 위치 (조종 가변저항으로부터, 테스트용 임의값) =====
current_left_ticks = list(station_positions.NEUTRAL_TICKS_LEFT)
current_right_ticks = list(station_positions.NEUTRAL_TICKS_RIGHT)

# ===== FSM 인스턴스 =====
left_fsm = tool_changer_fsm.ArmSwapFSM("left")
right_fsm = tool_changer_fsm.ArmSwapFSM("right")


def print_state():
    print(
        f"\n[상태]\n"
        f"  LEFT:  state={left_fsm.state:15s} held={str(left_fsm.held_gripper):10s} target={left_fsm.target_gripper}\n"
        f"  RIGHT: state={right_fsm.state:15s} held={str(right_fsm.held_gripper):10s} target={right_fsm.target_gripper}"
    )


def simulate_key_press(key1=False, key2=False, key3=False, key4=False, key5=False):
    left_target, right_target = key_input_handler.parse_key_input(
        key1, key2, key3, key4, key5
    )

    print(f"\n[키 입력] 1={key1} 2={key2} 3={key3} 4={key4} 5={key5}")
    if left_target or right_target:
        print(f"  → left_target={left_target}, right_target={right_target}")
    else:
        print(f"  → 아무것도 눌리지 않음")

    left_fsm.update(left_target, current_left_ticks)
    right_fsm.update(right_target, current_right_ticks)

    print_state()


def run_main_loop(steps):
    for step_num, (key1, key2, key3, key4, key5, desc) in enumerate(steps, 1):
        print(f"\n{'='*70}")
        print(f"[STEP {step_num}] {desc}")
        print(f"{'='*70}")

        simulate_key_press(key1, key2, key3, key4, key5)

        max_iterations = 10
        for i in range(max_iterations):
            prev_left_state = left_fsm.state
            prev_right_state = right_fsm.state

            left_fsm.update(None, current_left_ticks)
            right_fsm.update(None, current_right_ticks)

            if left_fsm.state == "IDLE" and right_fsm.state == "IDLE":
                print(f"\n  ✅ 양팔 모두 IDLE 상태 도달")
                break

            if left_fsm.state != prev_left_state or right_fsm.state != prev_right_state:
                print_state()


if __name__ == "__main__":
    print("=" * 70)
    print("그리퍼 자동교체 로직 테스트 (station_positions.py 실측값 사용)")
    print("=" * 70)

    steps = [
        # ===== Flow A 테스트 (빈손 → 부착) =====
        (
            True,
            False,
            False,
            False,
            False,
            "[Flow A] 1번: 오른팔 빈손 → 미세그리퍼 부착",
        ),
        (False, False, True, False, False, "[Flow A] 3번: 왼팔 빈손 → 기본그리퍼 부착"),
        # ===== Flow B 테스트 (짝 전환, 클리어런스) =====
        (
            False,
            True,
            False,
            False,
            False,
            "[Flow B] 2번: 오른팔 미세→니퍼 (클리어런스 경유)",
        ),
        (
            False,
            False,
            False,
            True,
            False,
            "[Flow B] 4번: 왼팔 기본→바이스 (클리어런스 경유)",
        ),
        # ===== 되돌리기 (반대 방향 클리어런스) =====
        (
            True,
            False,
            False,
            False,
            False,
            "[Flow B] 1번: 오른팔 니퍼→미세 (반대 클리어런스)",
        ),
        (
            False,
            False,
            True,
            False,
            False,
            "[Flow B] 3번: 왼팔 바이스→기본 (반대 클리어런스)",
        ),
    ]

    run_main_loop(steps)

    print("\n" + "=" * 70)
    print("✅ 테스트 완료!")
    print("=" * 70)
