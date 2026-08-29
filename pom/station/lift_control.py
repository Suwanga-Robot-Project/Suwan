"""
[라파 실제용] 상하이동(리프트) 모터 + 리미트 스위치를 gpiozero로 직접 제어.
lift_control_sim.py와 함수 이름/시그니처가 완전히 동일함.

⚠️ 실행 전 필수: 'sudo pigpiod' 불필요(gpiozero 기본 pin factory 사용),
   단 이 모듈은 라즈베리파이 본체에서만 동작합니다.

⚠️ 확인 필요: MOTOR_UP_PIN/MOTOR_DOWN_PIN이 실제로 상승/하강 중 어느 쪽인지,
   기존 w(상승)/s(하강)/x(정지)/q(종료) 수동 테스트 스크립트의 핀 매핑과
   대조해서 방향이 반대로 안 되어 있는지 반드시 재확인하세요.
"""

import time
from gpiozero import OutputDevice, Button

# ===== 모터 핀 (방향 제어만, PWM 아님) =====
MOTOR_UP_PIN = 17  # 확인 필요 — 기존 w/s/x/q 스크립트 매핑과 대조
MOTOR_DOWN_PIN = 27  # 확인 필요 — 기존 w/s/x/q 스크립트 매핑과 대조

# ===== 리미트 스위치 핀 (Pull-up + GND, gpiozero Button이 반전 자동처리) =====
TOP_SWITCH_PIN = 23  # 안전정지용
BOTTOM_SWITCH_PIN = 24  # 교체위치 감지용

DIRECTION_DEADTIME = 0.05  # 방향 전환 시 두 핀 모두 OFF 후 대기(안전 데드타임)
POLL_INTERVAL = 0.02  # 스위치 폴링 주기

motor_up = OutputDevice(MOTOR_UP_PIN)
motor_down = OutputDevice(MOTOR_DOWN_PIN)
top_switch = Button(TOP_SWITCH_PIN)
bottom_switch = Button(BOTTOM_SWITCH_PIN)


def _stop_motor():
    motor_up.off()
    motor_down.off()


def _start_descend():
    _stop_motor()
    time.sleep(DIRECTION_DEADTIME)
    motor_down.on()


def _start_ascend():
    _stop_motor()
    time.sleep(DIRECTION_DEADTIME)
    motor_up.on()


def descend_until_bottom_switch():
    """하단 리미트 스위치가 눌릴 때까지 하강. 걸린 시간(초)을 반환."""
    print(">>> 하강 시작...")
    start = time.time()
    _start_descend()

    while not bottom_switch.is_pressed:
        if top_switch.is_pressed:
            # 이상 상황: 하강 중인데 상단 스위치가 눌릴 리 없음 — 안전 정지
            _stop_motor()
            raise RuntimeError("하강 중 상단 리미트 스위치 감지 — 배선/로직 확인 필요")
        time.sleep(POLL_INTERVAL)

    _stop_motor()
    elapsed = time.time() - start
    print(f">>> 하단 리미트 스위치 도달! 정지 (경과 {elapsed:.2f}초)")
    return elapsed


def ascend_full(descend_seconds):
    """descend_seconds만큼 다시 상승 (시간기반 복귀). 상단 스위치는 안전 하드스톱."""
    print(f">>> {descend_seconds:.2f}초 동안 상승 시작...")
    start = time.time()
    _start_ascend()

    while time.time() - start < descend_seconds:
        if top_switch.is_pressed:
            _stop_motor()
            print(">>> 상단 리미트 스위치 도달 — 예정 시간 전 안전 정지")
            return
        time.sleep(POLL_INTERVAL)

    _stop_motor()
    print(">>> 상승 완료 (원래 위치로 복귀)")


def ascend_clearance(seconds):
    """짧게 살짝만 상승. 현재 arm_swap_sequence.py에서는 호출 안 함(수평이동만으로
    스왑 가능해져서) — 나중에 다시 필요해질 경우를 대비해 남겨둠."""
    print(f">>> {seconds:.2f}초 동안 살짝 상승(clearance)...")
    start = time.time()
    _start_ascend()
    while time.time() - start < seconds:
        if top_switch.is_pressed:
            _stop_motor()
            print(">>> 상단 리미트 스위치 도달 — clearance 중 안전 정지")
            return
        time.sleep(POLL_INTERVAL)
    _stop_motor()


if __name__ == "__main__":
    t = descend_until_bottom_switch()
    ascend_full(t)
