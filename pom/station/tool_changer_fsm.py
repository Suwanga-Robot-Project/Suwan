"""
그리퍼 자동교체 메인 상태머신.
왼팔/오른팔 각각 독립된 ArmSwapFSM 인스턴스로 관리하고, 리프트(상하이동)는
양팔 공용 축이라 별도의 참조카운트 기반 공유 관리자로 묶는다.

상태: IDLE → DESCEND → SWAP_EXECUTE → RETURN → DONE → IDLE

A안 (최종 확정):
- IDLE 상태일 때만 키 입력을 받는다
- DESCEND_SWAP/RETURN 도중 들어온 모든 키 입력은 무시된다
- 완전히 IDLE로 돌아와야만 다음 스왑 명령을 받는다

⚠️ 현재 구현은 동기(블로킹) 방식이다 — descend_until_bottom_switch() 등이
   끝날 때까지 메인 루프가 멈춘다.
"""

import time

try:
    import lift_control as lift_backend  # 라파 본체에서 직접 실행할 때만 동작 (GPIO 직결)

    print(">>> 로컬 GPIO(lift_control) 사용 — 라파 본체에서 실행 중")
except Exception:
    # ImportError뿐 아니라, gpiozero가 실제 GPIO 핀을 못 찾을 때 나는 에러도
    # 여기서 잡아야 함 (모듈 import 시점에 바로 핀 연결을 시도하는 구조라서)
    try:
        import lift_control_remote as lift_backend  # 노트북 → 라파 네트워크 원격 제어

        print(">>> 원격 라파 연결(lift_control_remote) 사용 — 노트북에서 실행 중")
    except Exception:
        import lift_control_sim as lift_backend  # 완전 가짜 (하드웨어 없이 테스트)

        print(">>> 시뮬레이션(lift_control_sim) 사용 — 상하이동 하드웨어 없음")

import arm_swap_sequence
import key_input_handler

# ===== 실제 서보 하드웨어 연결 시도 (없으면 콘솔 로그 모드로 자동 전환) =====
try:
    import servo_control

    servo_control.init_servos()
    _HARDWARE_AVAILABLE = True
    print(">>> 실제 서보 하드웨어 연결됨 — 팔이 실제로 움직입니다")
except Exception as e:
    _HARDWARE_AVAILABLE = False
    print(
        f">>> [경고] 서보 하드웨어 연결 실패 ({e}) — 콘솔 로그 모드로 동작 (팔 안 움직임)"
    )

# ===== 리프트(공용 축) 참조카운트 관리자 =====
_lift_users = 0
_lift_descend_seconds = None


def _request_lift_down():
    """이미 내려가 있으면(다른 팔이 쓰는 중이면) 실제 하강 없이 바로 통과."""
    global _lift_users, _lift_descend_seconds
    if _lift_users == 0:
        _lift_descend_seconds = lift_backend.descend_until_bottom_switch()
    _lift_users += 1


def _release_lift_down():
    """아직 다른 팔이 쓰는 중이면 상승 안 하고 카운트만 감소."""
    global _lift_users
    _lift_users -= 1
    if _lift_users <= 0:
        _lift_users = 0
        if _lift_descend_seconds is not None:
            lift_backend.ascend_full(_lift_descend_seconds)


def move_arm_to(arm_side, ticks):
    """
    팔을 목표 tick 배열로 이동.
    실제 하드웨어가 연결되어 있으면 servo_control.move_arm_to()로 진짜 이동시키고,
    없으면(PC 테스트 등) 콘솔에 로그만 출력.
    """
    tick_str = " ".join(f"{t:5d}" if t is not None else "  ?" for t in ticks)
    print(f"      [서보 이동] {arm_side}: {tick_str}")

    if _HARDWARE_AVAILABLE:
        servo_control.move_arm_to(arm_side, ticks)


class ArmSwapFSM:
    """한 팔의 그리퍼 자동교체 상태머신 (왼팔/오른팔 각각 인스턴스)."""

    def __init__(self, arm_side):
        self.arm_side = arm_side
        self.state = "IDLE"
        self.held_gripper = None
        self.target_gripper = None
        self.saved_arm_ticks = None

    def update(self, key_target, current_arm_ticks):
        """
        매 프레임 호출.

        key_target: 이번 프레임에 이 팔에 해당하는 목표 스테이션 이름
                    (예: 'default'/'vise'/'fine'/'nipper')
                    - None: 이번 프레임에 새 입력 없음 (아무것도 안 함)
                    - key_input_handler.DROP_ALL: 5번(전체탈거) — 빈손으로
        current_arm_ticks: 지금 이 팔의 실제 조종 tick (IDLE에서 스냅샷용)

        ⚠️ A안 규칙: IDLE이 아니면 key_target은 무시된다.
        """
        if self.state == "IDLE":
            if key_target is not None:
                # DROP_ALL이면 실제 target_gripper는 None(빈손)으로 변환
                actual_target = (
                    None if key_target == key_input_handler.DROP_ALL else key_target
                )
                if actual_target == self.held_gripper:
                    # 이미 그 상태임 (예: 빈손인데 또 전체탈거 요청) — 할 일 없으니 무시
                    return
                self.target_gripper = actual_target
                self.saved_arm_ticks = (
                    list(current_arm_ticks) if current_arm_ticks else None
                )
                self.state = "DESCEND_SWAP"
                print(
                    f"\n>>> [{self.arm_side} 팔] IDLE → DESCEND_SWAP (목표: {actual_target})"
                )
            return

        # ----- 이하는 IDLE이 아닌 상태 -----
        if self.state == "DESCEND_SWAP":
            _request_lift_down()
            time.sleep(1.0)  # 하강 완료 후 흔들림 안정화 대기

            # 그리퍼 교체 실행
            print(f">>> [{self.arm_side} 팔] 그리퍼 교체 시작")
            self.held_gripper = arm_swap_sequence.swap_gripper(
                self.arm_side, self.held_gripper, self.target_gripper, move_arm_to
            )
            print(
                f">>> [{self.arm_side} 팔] 그리퍼 교체 완료 (새 상태: {self.held_gripper})"
            )

            self.state = "RETURN"
            print(f">>> [{self.arm_side} 팔] DESCEND_SWAP → RETURN")

        elif self.state == "RETURN":
            _release_lift_down()
            print(f">>> [{self.arm_side} 팔] 원래 위치로 복귀")
            if self.saved_arm_ticks:
                move_arm_to(self.arm_side, self.saved_arm_ticks)

            self.state = "DONE"
            print(f">>> [{self.arm_side} 팔] RETURN → DONE")

        elif self.state == "DONE":
            self.state = "IDLE"
            self.target_gripper = None
            print(f">>> [{self.arm_side} 팔] DONE → IDLE (완료)\n")


# ===== 사용 예시 (test_tool_changer.py 메인 루프에 이런 식으로 연결) =====
if __name__ == "__main__":
    left_fsm = ArmSwapFSM("left")
    right_fsm = ArmSwapFSM("right")
    print("left_fsm/right_fsm 준비 완료")
    print("test_tool_changer.py 메인 루프에서 update() 호출하세요")
