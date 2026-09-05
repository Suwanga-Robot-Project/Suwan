import lgpio

# =====================================================
# [Wheel Control] BTS7960 x2 Driver Wiring Setup
# =====================================================
LEFT_RPWM_PIN = 12
LEFT_LPWM_PIN = 18
RIGHT_RPWM_PIN = 13
RIGHT_LPWM_PIN = 19

PWM_FREQ = 1000  

# =====================================================
# Joystick Calibration
# =====================================================
CENTER = 2033
DEADZONE = 150
MAX_DEV = 2000  

MIN_EFFECTIVE_DUTY = 0.2
MAX_DUTY = 1.0
MAX_ACCEL_PER_LOOP = 0.15  

# =====================================================
# Initialize lgpio 
# =====================================================
try:
    h = lgpio.gpiochip_open(0)
except Exception:
    h = lgpio.gpiochip_open(4) 

for pin in (LEFT_RPWM_PIN, LEFT_LPWM_PIN, RIGHT_RPWM_PIN, RIGHT_LPWM_PIN):
    try:
        lgpio.gpio_claim_output(h, pin)
    except Exception:
        pass

prev_left_duty = 0.0
prev_right_duty = 0.0

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

def _drive_pin_pair(rpwm_pin, lpwm_pin, duty, reversed_dir=False):
    if reversed_dir:
        rpwm_pin, lpwm_pin = lpwm_pin, rpwm_pin

    duty_percent = float(abs(duty) * 100.0)

    if duty > 0:
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, lpwm_pin, 0)
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, duty_percent)
    elif duty < 0:
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, rpwm_pin, 0)
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, duty_percent)
    else:
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, 0.0)
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, rpwm_pin, 0)
        lgpio.gpio_write(h, lpwm_pin, 0)

def update_wheels(parsed, sw1):
    global prev_left_duty, prev_right_duty

    if sw1 != 1:
        _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0)
        _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0, reversed_dir=True)
        prev_left_duty = 0.0
        prev_right_duty = 0.0
        return

    throttle = _norm(parsed[17])
    turn = _norm(parsed[18])

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

def _drive_pin_pair(rpwm_pin, lpwm_pin, duty, reversed_dir=False):
    if reversed_dir:
        rpwm_pin, lpwm_pin = lpwm_pin, rpwm_pin

    duty_percent = float(abs(duty) * 100.0)

    if duty > 0:
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, lpwm_pin, 0)
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, duty_percent)
    elif duty < 0:
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, rpwm_pin, 0)
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, duty_percent)
    else:
        lgpio.tx_pwm(h, rpwm_pin, PWM_FREQ, 0.0)
        lgpio.tx_pwm(h, lpwm_pin, PWM_FREQ, 0.0)
        lgpio.gpio_write(h, rpwm_pin, 0)
        lgpio.gpio_write(h, lpwm_pin, 0)

def update_wheels(parsed, sw1):
    global prev_left_duty, prev_right_duty

    if sw1 != 1:
        _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0)
        _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0, reversed_dir=True)
        prev_left_duty = 0.0
        prev_right_duty = 0.0
        return

    throttle = _norm(parsed[17])
    turn = _norm(parsed[18])

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
    global prev_left_duty, prev_right_duty
    _drive_pin_pair(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0)
    _drive_pin_pair(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0, reversed_dir=True)
    prev_left_duty = 0.0
    prev_right_duty = 0.0
