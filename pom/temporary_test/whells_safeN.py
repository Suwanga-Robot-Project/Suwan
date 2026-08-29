# =====================================================
# [테스트 전용] 라파 없이 PC에서 바퀴 제어 알고리즘만 검증하는 버전
# 실제 GPIO/pigpio를 전혀 안 건드리고, 계산된 duty/방향을 콘솔에 출력만 함
# wheels_safe.py(라파 실제 구동용)와 update_wheels() 시그니처가 동일해서
# Nexus_5.py 쪽 코드는 손대지 않고 import만 바꿔서 테스트 가능
# =====================================================

# =====================================================
# 조이스틱 캘리브레이션 (실측 기준, wheels_safe.py와 동일)
# =====================================================
CENTER = 1550
DEADZONE = 150
MAX_DEV = 1550

MIN_EFFECTIVE_DUTY = 0.2
MAX_DUTY = 1.0

MAX_ACCEL_PER_LOOP = 0.15

prev_left_duty = 0.0
prev_right_duty = 0.0

# 콘솔 출력 스팸 방지 — 값이 의미 있게 바뀔 때만 찍음
_last_printed_left = None
_last_printed_right = None
_PRINT_THRESHOLD = 0.02


def _norm(raw):
    dev = raw - CENTER
    if abs(dev) <= DEADZONE:
        return 0.0
    sign = 1.0 if dev > 0 else -1.0
    magnitude = (abs(dev) - DEADZONE) / (MAX_DEV - DEADZONE)
    return sign * max(0.0, min(1.0, magnitude))


def _apply_min_duty(duty):
    if duty == 0:
        return 0.0
    sign = 1.0 if duty > 0 else -1.0
    mag = max(abs(duty), MIN_EFFECTIVE_DUTY)
    return sign * min(mag, MAX_DUTY)


def _ramp(prev, target, max_step):
    if prev == 0.0 and target != 0.0:
        if abs(target) <= MIN_EFFECTIVE_DUTY:
            return target
        start = MIN_EFFECTIVE_DUTY if target > 0 else -MIN_EFFECTIVE_DUTY
        diff = target - start
        step = max(-max_step, min(max_step, diff))
        return start + step
    diff = target - prev
    step = max(-max_step, min(max_step, diff))
    return prev + step


def _direction_label(duty):
    if duty > 0.01:
        return f"전진 {duty*100:5.1f}%"
    elif duty < -0.01:
        return f"후진 {abs(duty)*100:5.1f}%"
    else:
        return "정지        "


def update_wheels(parsed, sw1):
    """
    wheels_safe.py와 완전히 동일한 인터페이스.
    실제 핀 구동 대신 콘솔에 왼쪽/오른쪽 duty와 방향만 출력.
    """
    global prev_left_duty, prev_right_duty
    global _last_printed_left, _last_printed_right

    if sw1 != 1:
        if prev_left_duty != 0.0 or prev_right_duty != 0.0:
            print(">>> [바퀴 시뮬] SW1=0 — 정지")
        prev_left_duty = 0.0
        prev_right_duty = 0.0
        _last_printed_left = 0.0
        _last_printed_right = 0.0
        return

    throttle = _norm(parsed[17])
    turn = _norm(parsed[18])

    left_target = max(-1.0, min(1.0, throttle + turn))
    right_target = max(-1.0, min(1.0, throttle - turn))

    left_target = _apply_min_duty(left_target)
    right_target = _apply_min_duty(right_target)

    left_duty = _ramp(prev_left_duty, left_target, MAX_ACCEL_PER_LOOP)
    right_duty = _ramp(prev_right_duty, right_target, MAX_ACCEL_PER_LOOP)

    prev_left_duty = left_duty
    prev_right_duty = right_duty

    # 의미 있게 바뀔 때만 출력 (매 루프 20ms마다 찍으면 스팸이라)
    if (
        _last_printed_left is None
        or abs(left_duty - _last_printed_left) > _PRINT_THRESHOLD
        or abs(right_duty - _last_printed_right) > _PRINT_THRESHOLD
    ):
        print(
            f">>> [바퀴 시뮬] throttle={throttle:+.2f} turn={turn:+.2f}  |  "
            f"L: {_direction_label(left_duty)}  R: {_direction_label(right_duty)}"
        )
        _last_printed_left = left_duty
        _last_printed_right = right_duty


def stop_all():
    global prev_left_duty, prev_right_duty
    print(">>> [바퀴 시뮬] stop_all() 호출됨 — 정지")
    prev_left_duty = 0.0
    prev_right_duty = 0.0
