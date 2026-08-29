"""
[PC 테스트용] 라파/GPIO 없이 상하이동을 흉내내는 시뮬레이션 모듈.
lift_control.py와 함수 이름/시그니처가 완전히 동일해서, import 한 줄만
바꾸면 실제 라파용으로 교체 가능.

하단 리미트 스위치 도달 시점은 사람이 엔터를 눌러서 직접 알려주는 방식으로
흉내낸다 (실제 스위치가 눌리는 순간을 사람이 대신 신호).
"""

import time

_last_descend_seconds = None


def descend_until_bottom_switch():
    """하단 리미트 스위치가 눌릴 때까지 하강(시뮬). 걸린 시간(초)을 반환."""
    global _last_descend_seconds
    print(">>> [시뮬] 하강 시작... (하단 스위치가 눌렸다고 가정되면 엔터를 누르세요)")
    start = time.time()
    input()
    elapsed = time.time() - start
    _last_descend_seconds = elapsed
    print(f">>> [시뮬] 하단 리미트 스위치 도달! 정지 (경과 {elapsed:.2f}초)")
    return elapsed


def ascend_full(descend_seconds):
    """descend_seconds만큼 다시 상승(시뮬), 저장된 시간만큼 되돌아가는 시간기반 방식."""
    print(f">>> [시뮬] {descend_seconds:.2f}초 동안 상승 시작...")
    time.sleep(
        min(descend_seconds, 2.0)
    )  # 시뮬에서는 실제로 그만큼 안 기다리고 최대 2초만 대기
    print(">>> [시뮬] 상승 완료 (원래 위치로 복귀 가정)")


def ascend_clearance(seconds):
    """짧게 살짝만 상승(시뮬). 현재 arm_swap_sequence.py에서는 호출 안 함(수평이동만으로
    스왑 가능해져서) — 나중에 다시 필요해질 경우를 대비해 남겨둠."""
    print(f">>> [시뮬] {seconds:.2f}초 동안 살짝 상승(clearance)...")
    time.sleep(seconds)
    print(">>> [시뮬] clearance 상승 완료")


if __name__ == "__main__":
    # 단독 실행 시 동작 확인용
    t = descend_until_bottom_switch()
    ascend_full(t)
