import pigpio

# =====================================================
# [바퀴 제어] BTS7960 x2 — 하드웨어팀 확정 배선 기준
# 왼쪽: RPWM=GPIO12, LPWM=GPIO18
# 오른쪽: RPWM=GPIO13, LPWM=GPIO19
# LEN/REN은 배선에서 상시 HIGH 고정 — 여기서 다룰 필요 없음
# =====================================================
LEFT_RPWM_PIN = 12
LEFT_LPWM_PIN = 18
RIGHT_RPWM_PIN = 13
RIGHT_LPWM_PIN = 19

PWM_FREQ = 1000  # Hz — 하드웨어팀 "자유롭게 정해도 됨" 확인, 필요시 조정

# =====================================================
# 조이스틱 캘리브레이션 (실측 기준)
# parsed[17] = PC1(ind2) = 전후진, parsed[18] = PA4(ind3) = 좌우회전
# =====================================================
CENTER = 1550
DEADZONE = 150
MAX_DEV = 1550  # 중립에서 최대 편차 → duty 100%로 매핑

# =====================================================
# 하중 데드존 — 이 밑으로는 바퀴가 안 돎 (실측: 대략 0.0~0.2)
# =====================================================
MIN_EFFECTIVE_DUTY = 0.2
MAX_DUTY = 1.0

# =====================================================
# 목표값 자체의 무시 임계값 — 이보다 작은 목표는 "사실상 0"으로 취급.
# (없으면, 아주 작은 노이즈/중심값 오차 하나만 있어도 _apply_min_duty가
#  그걸 무조건 MIN_EFFECTIVE_DUTY까지 증폭시켜서 실제로 눈에 보이게
#  움직여버림 — 이 임계값이 그 증폭을 막아주는 역할)
# =====================================================
TARGET_IGNORE_THRESHOLD = 0.03

# =====================================================
# 원인 진단용 디버그 로그 — 필요없으면 False로 끄면 됨
# =====================================================
DEBUG_PRINT = True
DEBUG_PRINT_INTERVAL = 20  # 몇 프레임마다 한 번 찍을지 (너무 자주 찍히면 스팸)
_debug_counter = 0

# =====================================================
# 가속도 제한 — 급가속/급정거 방지, 튜닝 필요할 수 있음
# =====================================================
MAX_ACCEL_PER_LOOP = 0.15  # 한 루프(제어 주기)당 duty 최대 변화량

# =====================================================
# pigpio 초기화 — 라즈베리파이에서 반드시 pigpiod 데몬이 먼저 떠 있어야 함
#   sudo pigpiod
# =====================================================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError(
        "pigpio 데몬에 연결 실패 — 라즈베리파이에서 'sudo pigpiod' 먼저 실행하세요"
    )

prev_left_duty = 0.0
prev_right_duty = 0.0


def _norm(raw):
    """조이스틱 raw ADC 값을 -1.0(최대후진/좌) ~ +1.0(최대전진/우)으로 정규화."""
    dev = raw - CENTER
    if abs(dev) <= DEADZONE:
        return 0.0
    sign = 1.0 if dev > 0 else -1.0
    magnitude = (abs(dev) - DEADZONE) / (MAX_DEV - DEADZONE)
    return sign * max(0.0, min(1.0, magnitude))


def _apply_min_duty(duty):
    """
    0이 아닌데 MIN_EFFECTIVE_DUTY보다 작으면 하중 데드존을 건너뛰도록 끌어올림.
    단, TARGET_IGNORE_THRESHOLD보다도 작은 값(노이즈 수준)은 애초에 0으로
    무시해서, 미세한 오차가 증폭되어 눈에 보이게 움직이는 걸 방지한다.
    """
    if abs(duty) <= TARGET_IGNORE_THRESHOLD:
        return 0.0
    sign = 1.0 if duty > 0 else -1.0
    mag = max(abs(duty), MIN_EFFECTIVE_DUTY)
    return sign * min(mag, MAX_DUTY)


def _ramp(prev, target, max_step):
    """가속도 제한. 단, 0에서 출발할 때는 하중 데드존(MIN_EFFECTIVE_DUTY)까지는
    즉시 건너뛰고, 그 이상 구간만 램프를 적용한다."""
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


def _drive_pin_pair(rpwm_pin, lpwm_pin, duty, reversed_dir=False):
    """
    duty: -1.0(후진) ~ +1.0(전진)
    반대 핀을 먼저 0으로 내린 뒤 활성 핀에 duty를 준다 (동시 duty 절대 금지).
    reversed_dir=True면 RPWM/LPWM 역할을 서로 바꿔서 물리적 반전을 보정한다.
    """
    if reversed_dir:
        rpwm_pin, lpwm_pin = lpwm_pin, rpwm_pin

    duty_val = int(abs(duty) * 1_000_000)  # pigpio hardware_PWM은 0~1,000,000 범위

    if duty > 0:
        pi.hardware_PWM(lpwm_pin, 0, 0)
        pi.write(lpwm_pin, 0)
        pi.hardware_PWM(rpwm_pin, PWM_FREQ, duty_val)
    elif duty < 0:
        pi.hardware_PWM(rpwm_pin, 0, 0)
        pi.write(rpwm_pin, 0)
        pi.hardware_PWM(lpwm_pin, PWM_FREQ, duty_val)
    else:
        pi.hardware_PWM(rpwm_pin, 0, 0)
        pi.hardware_PWM(lpwm_pin, 0, 0)
        pi.write(rpwm_pin, 0)
        pi.write(lpwm_pin, 0)


def update_wheels(parsed, sw1):
    """
    parsed[17] = 왼쪽 조이스틱 ind2 (PC1, 전후진)
    parsed[18] = 왼쪽 조이스틱 ind3 (PA4, 좌우회전)
    sw1: sw1_toggle 값 — 1일 때만 유효 입력으로 처리, 그 외엔 무조건 정지.
    """
    global prev_left_duty, prev_right_duty

    if sw1 != 1:
        _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0)
        _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0, reversed_dir=True)
        prev_left_duty = 0.0
        prev_right_duty = 0.0
        return

    throttle = _norm(parsed[17])
    turn = _norm(parsed[18])

    global _debug_counter
    if DEBUG_PRINT:
        _debug_counter += 1
        if _debug_counter % DEBUG_PRINT_INTERVAL == 0:
            print(
                f"[바퀴 디버그] raw17(전후진)={parsed[17]:4d} raw18(회전)={parsed[18]:4d} "
                f"| throttle={throttle:+.3f} turn={turn:+.3f}"
            )

    left_target = max(-1.0, min(1.0, throttle + turn))
    right_target = max(-1.0, min(1.0, throttle - turn))

    left_target = _apply_min_duty(left_target)
    right_target = _apply_min_duty(right_target)

    left_duty = _ramp(prev_left_duty, left_target, MAX_ACCEL_PER_LOOP)
    right_duty = _ramp(prev_right_duty, right_target, MAX_ACCEL_PER_LOOP)

    _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, left_duty)
    _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, right_duty, reversed_dir=True)

    prev_left_duty = left_duty
    prev_right_duty = right_duty


def stop_all():
    """비상/종료 시 양쪽 바퀴 완전 정지."""
    global prev_left_duty, prev_right_duty
    _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0)
    _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0, reversed_dir=True)
    prev_left_duty = 0.0
    prev_right_duty = 0.0
