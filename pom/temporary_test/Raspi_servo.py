import serial
import threading
import time
import struct  # CRC
import socket  # UDP 통신용
import pantilt_safe3
from pantilt_safe3 import update_pantilt

# =====================================================
# [PC용] 이 스크립트는 라즈베리파이의 servo_receiver.py와 짝을 이룹니다.
# PC는 STM32 ADC(COM13)만 로컬로 읽고, 연산(FSM/이상탐지/EMA/tick계산)을
# 전부 마친 뒤 최종 tick 값을 UDP로 라파에 보냅니다.
# 서보(왼팔/오른팔/팬틸트)는 전부 라파에 물려있어 여기서는 직접 제어하지 않습니다.
# =====================================================

# =====================================================
# [공통] FSM 상태 정의
# IDLE  : 해당 팔 전 채널 정지 상태, 모터 명령 안 보냄
# MOVE  : 해당 팔 최소 1채널 이상 움직이는 정상 동작 상태
# ERROR : 이상 감지 시 진입, 해당 팔 tick 값 고정(더 이상 갱신 안 함)
# 왼팔/오른팔은 서로 독립된 FSM 상태를 가짐
# =====================================================
STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

IDLE_CONFIRM_LOOPS = 10  # 약 0.2초(루프 0.02s 기준), 채터링 방지용 debounce

# =====================================================
# Stage 1/2 이상탐지 파라미터
# =====================================================
ADC_MIN_VALID = 0
ADC_MAX_VALID = 4095

MAX_RAW_DELTA = 1500  # Stage 2: 한 프레임 최대 허용 변화량 (raw ADC 스케일)
EXTREME_LOW = 30  # Stage 2: 극단값(하한) 기준 — 단선 패턴 감지용
EXTREME_HIGH = 4065  # Stage 2: 극단값(상한) 기준
ANOMALY_CONFIRM_COUNT = 3  # 연속 이상 프레임 수 → ERROR 전환 기준

# 채널별 이전 프레임 raw 값 / 연속 이상 카운트 (왼팔 7채널, 오른팔 7채널)
prev_raw_left = [None] * 7
prev_raw_right = [None] * 7
anomaly_count_left = [0] * 7
anomaly_count_right = [0] * 7


# =====================================================
# CRC-16 CCITT (직접 구현)
# 다항식: 0x1021, 초기값: 0xFFFF (CCITT-FALSE 변형)
# =====================================================
def calc_crc16_ccitt(data: bytes, initial=0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_self_test():
    """
    표준 CRC-16/CCITT-FALSE 검증 벡터: "123456789" → 0x29B1
    """
    test_data = b"123456789"
    result = calc_crc16_ccitt(test_data)
    expected = 0x29B1
    status = "PASS" if result == expected else "FAIL"
    print(
        f">>> [CRC16 자체테스트] 계산값=0x{result:04X} 기대값=0x{expected:04X} → {status}"
    )


# =====================================================
# 시퀀스 넘버 검증
# =====================================================
class SequenceValidator:
    def __init__(self, max_seq=65535):
        self.last_seq = None
        self.max_seq = max_seq
        self.lost_count = 0
        self.duplicate_count = 0
        self.reorder_count = 0

    def check(self, seq):
        if self.last_seq is None:
            self.last_seq = seq
            return "OK"

        expected = (self.last_seq + 1) % (self.max_seq + 1)

        if seq == expected:
            result = "OK"
        elif seq == self.last_seq:
            result = "DUPLICATE"
            self.duplicate_count += 1
        elif self._is_ahead(seq, expected):
            result = "LOST"
            self.lost_count += 1
        else:
            result = "REORDER"
            self.reorder_count += 1

        if result in ("OK", "LOST"):
            self.last_seq = seq

        return result

    def _is_ahead(self, seq, expected):
        diff = (seq - expected) % (self.max_seq + 1)
        return diff < (self.max_seq // 2)


PACKET_HEADER = b"\xaa\x55"
PACKET_SIZE = 51
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBH")

seq_checker = SequenceValidator()


def sequence_validator_self_test():
    print(">>> [시퀀스 검증 자체테스트]")
    cases = [
        ("정상", [1, 2, 3, 4, 5], ["OK", "OK", "OK", "OK", "OK"]),
        ("유실", [1, 2, 4, 5], ["OK", "OK", "LOST", "OK"]),
        ("중복", [1, 2, 2, 3], ["OK", "OK", "DUPLICATE", "OK"]),
        ("뒤바뀜", [1, 3, 2, 4], ["OK", "LOST", "REORDER", "OK"]),
    ]
    for name, seq_list, expected_list in cases:
        v = SequenceValidator()
        results = [v.check(s) for s in seq_list]
        status = "PASS" if results == expected_list else "FAIL"
        print(
            f"    {name}: seq={seq_list} → {results} (기대:{expected_list}) → {status}"
        )


def check_anomaly(channel_parsed, prev_raw, anomaly_count, arm_name):
    """
    Stage 1: 절대범위(0~4095) 이탈 → 즉시 이상 판정
    Stage 2: 변화율 급변 또는 극단값 왕복(단선 패턴) → 채널별 연속 카운트,
            ANOMALY_CONFIRM_COUNT(3회) 연속되면 True(ERROR) 리턴
    """
    error_triggered = False

    for i in range(7):
        raw = channel_parsed[i]

        if raw < ADC_MIN_VALID or raw > ADC_MAX_VALID:
            print(f">>> [Stage1] {arm_name} 채널{i} 범위 이탈 raw={raw}")
            error_triggered = True
            continue

        is_anomaly_frame = False

        if prev_raw[i] is not None:
            delta = abs(raw - prev_raw[i])

            if delta > MAX_RAW_DELTA:
                is_anomaly_frame = True

            prev_near_low = prev_raw[i] <= EXTREME_LOW
            curr_near_low = raw <= EXTREME_LOW
            prev_near_high = prev_raw[i] >= EXTREME_HIGH
            curr_near_high = raw >= EXTREME_HIGH

            if (prev_near_low and curr_near_high) or (prev_near_high and curr_near_low):
                is_anomaly_frame = True

        if is_anomaly_frame:
            anomaly_count[i] += 1
            if anomaly_count[i] >= ANOMALY_CONFIRM_COUNT:
                print(
                    f">>> [Stage2] {arm_name} 채널{i} 연속 이상 {anomaly_count[i]}회 감지"
                )
                error_triggered = True
        else:
            anomaly_count[i] = 0

        prev_raw[i] = raw

    return error_triggered


# =====================================================
# 상하이동(리프트) 3구간 상태 판정 — 히스테리시스 적용
# =====================================================
LIFT_LOW_ENTER = 1400
LIFT_LOW_EXIT = 1600
LIFT_HIGH_ENTER = 2200
LIFT_HIGH_EXIT = 2000
LIFT_REVERSED = False

lift_state = 0


def update_lift_state(raw_adc):
    global lift_state

    if LIFT_REVERSED:
        raw_adc = 4095 - raw_adc

    if lift_state == 0:
        if raw_adc < LIFT_LOW_ENTER:
            lift_state = -1
        elif raw_adc > LIFT_HIGH_ENTER:
            lift_state = 1
    elif lift_state == -1:
        if raw_adc > LIFT_LOW_EXIT:
            lift_state = 0
    elif lift_state == 1:
        if raw_adc < LIFT_HIGH_EXIT:
            lift_state = 0

    return lift_state


# =========================
# STM32 ADC 시리얼 (PC 로컬 — 양팔 공용, 스레드 1개만 실행)
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

# =====================================================
# 라즈베리파이 UDP 전송 설정
# =====================================================
RPI_IP = "192.168.0.24"
RPI_PORT = 5005
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

adc_raw = [0] * 23
parsed = [0] * 17  # [0:7]=왼팔, [7:14]=오른팔, [14:16]=조이스틱IND, [16]=상하이동

sw_toggle = 0
running = True

FLOATING_THRESHOLD = 4080

SERIAL_TIMEOUT_SEC = 0.5
last_serial_rx_time = time.time()


def read_serial_adc():
    global adc_raw, parsed, sw_toggle, running, last_serial_rx_time

    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    except Exception as e:
        print("시리얼 열기 실패:", e)
        return

    buf = bytearray()

    while running:
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                idx = buf.find(PACKET_HEADER)

                if idx == -1:
                    if len(buf) > 1:
                        del buf[:-1]
                    break

                if idx > 0:
                    del buf[:idx]

                if len(buf) < PACKET_SIZE:
                    break

                raw_packet = bytes(buf[:PACKET_SIZE])

                (
                    header,
                    msg_type,
                    seq_num,
                    mux0,
                    mux1,
                    mux2,
                    mux3,
                    mux4,
                    mux5,
                    mux6,
                    mux7,
                    mux8,
                    mux9,
                    mux10,
                    mux11,
                    mux12,
                    mux13,
                    mux14,
                    mux15,
                    ind0,
                    ind1,
                    ind2,
                    ind3,
                    ind4,
                    sw0,
                    sw1,
                    crc_received,
                ) = PACKET_STRUCT.unpack(raw_packet)

                crc_calc = calc_crc16_ccitt(raw_packet[: PACKET_SIZE - 2])

                if crc_calc != crc_received:
                    print(
                        f">>> [CRC 오류] seq={seq_num} 계산=0x{crc_calc:04X} 수신=0x{crc_received:04X} → 패킷 폐기"
                    )
                    del buf[:2]
                    continue

                seq_result = seq_checker.check(seq_num)
                if seq_result != "OK":
                    print(f">>> [시퀀스] seq={seq_num} → {seq_result}")

                mux_adc_vals = [
                    mux0,
                    mux1,
                    mux2,
                    mux3,
                    mux4,
                    mux5,
                    mux6,
                    mux7,
                    mux8,
                    mux9,
                    mux10,
                    mux11,
                    mux12,
                    mux13,
                    mux14,
                    mux15,
                ]

                for i in range(16):
                    adc_raw[i] = mux_adc_vals[i]
                adc_raw[16] = ind0
                adc_raw[17] = ind1
                adc_raw[18] = ind2
                adc_raw[19] = ind3
                adc_raw[20] = ind4
                adc_raw[21] = sw0
                adc_raw[22] = sw1

                for i in range(7):
                    parsed[i] = adc_raw[i + 1]
                for i in range(7):
                    parsed[i + 7] = adc_raw[i + 9]
                parsed[14] = adc_raw[16]
                parsed[15] = adc_raw[17]
                parsed[16] = adc_raw[20]
                sw_toggle = adc_raw[21]

                last_serial_rx_time = time.time()

                del buf[:PACKET_SIZE]

        except Exception as e:
            print("ERR:", e)


# =========================
# 왼팔 tick 계산 설정 (로컬 서보 없음 — UDP로 보낼 값만 계산)
# =========================
MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]

EMA_ALPHA_ARM_LEFT = [0.35, 0.35, 0.3, 0.3, 0.3, 0.7]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 28
DEAD_ZONE_EXIT_LEFT = 40
MAX_DELTA_LEFT = 70
MAX_ACCEL_LEFT = 15

GRIPPER_ADC_MIN_LEFT = 145
GRIPPER_ADC_MAX_LEFT = 1270
GRIPPER_POS_OPEN_LEFT = 4100
GRIPPER_POS_CLOSE_LEFT = 500

ema_values_left = [None] * 7
prev_ticks_left = [None] * 7
prev_delta_left = [0] * 7
in_dead_zone_left = [False] * 7
dead_zone_anchor_left = [None] * 7
current_state_left = STATE_IDLE
prev_state_left = STATE_IDLE
idle_confirm_count_left = 0

system_ready_left = False
startup_count_left = 0
STARTUP_WAIT_LEFT = 80

# =========================
# 오른팔 + 팬틸트 tick 계산 설정
# =========================
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
REVERSE_CHANNELS_RIGHT = [0, 3, 4, 5, 6]

PAN_ID = 22
TILT_ID = 33

EMA_ALPHA_ARM_RIGHT = [0.35, 0.35, 0.3, 0.3, 0.3, 0.7]
EMA_ALPHA_GRIPPER_RIGHT = 0.5

DEAD_ZONE_ENTER_RIGHT = 28
DEAD_ZONE_EXIT_RIGHT = 40
MAX_DELTA_RIGHT = 70
MAX_ACCEL_RIGHT = 15

GRIPPER_ADC_MIN_RIGHT = 2973
GRIPPER_ADC_MAX_RIGHT = 3993
GRIPPER_POS_OPEN_RIGHT = 3935
GRIPPER_POS_CLOSE_RIGHT = 0

ema_values_right = [None] * 7
prev_ticks_right = [None] * 7
prev_delta_right = [0] * 7
in_dead_zone_right = [False] * 7
dead_zone_anchor_right = [None] * 7
current_state_right = STATE_IDLE
prev_state_right = STATE_IDLE
idle_confirm_count_right = 0

system_ready_right = False
startup_count_right = 0
STARTUP_WAIT_RIGHT = 50


# =========================
# 왼팔 tick 계산 함수
# ERROR 상태에서는 로컬 토크 OFF 대신 tick 값을 마지막 정상값으로 고정
# (라파 쪽에서 진짜 정지/토크OFF가 필요하면 별도 프로토콜 확장 필요)
# =========================
def process_left_arm():
    global ema_values_left, prev_ticks_left, in_dead_zone_left
    global current_state_left, idle_confirm_count_left, prev_state_left
    global dead_zone_anchor_left

    if check_anomaly(parsed[0:7], prev_raw_left, anomaly_count_left, "왼팔"):
        current_state_left = STATE_ERROR

    if current_state_left == STATE_ERROR:
        if prev_state_left != STATE_ERROR:
            print(
                ">>> ERROR: 왼팔 이상 감지 — tick 값 고정, 라파에 마지막 정상값만 계속 전송됨"
            )
        prev_state_left = current_state_left
        return

    prev_state_left = current_state_left

    all_idle = all(
        prev_ticks_left[i] is None
        or in_dead_zone_left[i]
        or parsed[i] >= FLOATING_THRESHOLD
        for i in range(7)
    )
    if all_idle:
        idle_confirm_count_left += 1
        if idle_confirm_count_left >= IDLE_CONFIRM_LOOPS:
            current_state_left = STATE_IDLE
    else:
        idle_confirm_count_left = 0
        current_state_left = STATE_MOVE

    for i in range(7):
        raw = parsed[i]

        if raw >= FLOATING_THRESHOLD:
            continue

        alpha = EMA_ALPHA_GRIPPER_LEFT if i == 6 else EMA_ALPHA_ARM_LEFT[i]

        if ema_values_left[i] is None:
            ema_values_left[i] = float(raw)
        else:
            ema_values_left[i] = alpha * raw + (1 - alpha) * ema_values_left[i]

        if i == 6:
            adc = int(ema_values_left[6])
            ratio = max(
                0.0,
                min(
                    1.0,
                    (adc - GRIPPER_ADC_MIN_LEFT)
                    / (GRIPPER_ADC_MAX_LEFT - GRIPPER_ADC_MIN_LEFT),
                ),
            )
            tick = int(
                GRIPPER_POS_CLOSE_LEFT
                + ratio * (GRIPPER_POS_OPEN_LEFT - GRIPPER_POS_CLOSE_LEFT)
            )
        else:
            tick = int(ema_values_left[i])
            if i in REVERSE_CHANNELS_LEFT:
                tick = 4095 - tick

        if prev_ticks_left[i] is not None:
            delta = tick - prev_ticks_left[i]
            if delta > MAX_DELTA_LEFT:
                tick = prev_ticks_left[i] + MAX_DELTA_LEFT
            elif delta < -MAX_DELTA_LEFT:
                tick = prev_ticks_left[i] - MAX_DELTA_LEFT

            actual_delta = tick - prev_ticks_left[i]
            accel = actual_delta - prev_delta_left[i]
            if accel > MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] + MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            elif accel < -MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] - MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            prev_delta_left[i] = actual_delta

            if i != 6:
                if i in REVERSE_CHANNELS_LEFT:
                    ema_values_left[i] = float(4095 - tick)
                else:
                    ema_values_left[i] = float(tick)

        if prev_ticks_left[i] is not None:
            diff = abs(tick - prev_ticks_left[i])
            if in_dead_zone_left[i]:
                diff_from_anchor = abs(tick - dead_zone_anchor_left[i])
                if diff_from_anchor <= DEAD_ZONE_EXIT_LEFT:
                    continue
                else:
                    in_dead_zone_left[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_LEFT:
                    in_dead_zone_left[i] = True
                    dead_zone_anchor_left[i] = tick
                    continue

        prev_ticks_left[i] = tick


# =========================
# 오른팔 tick 계산 함수
# =========================
def process_right_arm():
    global ema_values_right, prev_ticks_right, in_dead_zone_right
    global current_state_right, idle_confirm_count_right, prev_state_right
    global dead_zone_anchor_right

    if check_anomaly(parsed[7:14], prev_raw_right, anomaly_count_right, "오른팔"):
        current_state_right = STATE_ERROR

    if current_state_right == STATE_ERROR:
        if prev_state_right != STATE_ERROR:
            print(
                ">>> ERROR: 오른팔 이상 감지 — tick 값 고정, 라파에 마지막 정상값만 계속 전송됨"
            )
        prev_state_right = current_state_right
        return

    prev_state_right = current_state_right

    all_idle = all(
        prev_ticks_right[i] is None or in_dead_zone_right[i] for i in range(7)
    )
    if all_idle:
        idle_confirm_count_right += 1
        if idle_confirm_count_right >= IDLE_CONFIRM_LOOPS:
            current_state_right = STATE_IDLE
    else:
        idle_confirm_count_right = 0
        current_state_right = STATE_MOVE

    for i in range(7):
        raw = parsed[i + 7]

        alpha = EMA_ALPHA_GRIPPER_RIGHT if i == 6 else EMA_ALPHA_ARM_RIGHT[i]

        if ema_values_right[i] is None:
            ema_values_right[i] = float(raw)
        else:
            ema_values_right[i] = alpha * raw + (1 - alpha) * ema_values_right[i]

        if i == 6:
            adc = int(ema_values_right[6])
            ratio = max(
                0.0,
                min(
                    1.0,
                    (adc - GRIPPER_ADC_MIN_RIGHT)
                    / (GRIPPER_ADC_MAX_RIGHT - GRIPPER_ADC_MIN_RIGHT),
                ),
            )
            tick = int(
                GRIPPER_POS_CLOSE_RIGHT
                + ratio * (GRIPPER_POS_OPEN_RIGHT - GRIPPER_POS_CLOSE_RIGHT)
            )
        else:
            tick = int(ema_values_right[i])
            if i in REVERSE_CHANNELS_RIGHT:
                tick = 4095 - tick

        if prev_ticks_right[i] is not None:
            delta = tick - prev_ticks_right[i]
            if delta > MAX_DELTA_RIGHT:
                tick = prev_ticks_right[i] + MAX_DELTA_RIGHT
            elif delta < -MAX_DELTA_RIGHT:
                tick = prev_ticks_right[i] - MAX_DELTA_RIGHT

            actual_delta = tick - prev_ticks_right[i]
            accel = actual_delta - prev_delta_right[i]
            if accel > MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] + MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            elif accel < -MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] - MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            prev_delta_right[i] = actual_delta

            if i != 6:
                if i in REVERSE_CHANNELS_RIGHT:
                    ema_values_right[i] = float(4095 - tick)
                else:
                    ema_values_right[i] = float(tick)

            diff = abs(tick - prev_ticks_right[i])
            if in_dead_zone_right[i]:
                diff_from_anchor = abs(tick - dead_zone_anchor_right[i])
                if diff_from_anchor <= DEAD_ZONE_EXIT_RIGHT:
                    continue
                else:
                    in_dead_zone_right[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_RIGHT:
                    in_dead_zone_right[i] = True
                    dead_zone_anchor_right[i] = tick
                    continue

        prev_ticks_right[i] = tick


# =========================
# ADC thread (PC 로컬, 1개만 실행)
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

crc16_self_test()
sequence_validator_self_test()

print("시작 — 서보는 라파에서 구동됨, 이 PC는 연산+UDP송신만 담당")

# =========================
# 메인 루프
# =========================
try:
    while True:

        if not (system_ready_left and system_ready_right):
            if not system_ready_left:
                startup_count_left += 1
                if startup_count_left % 50 == 0:
                    print(
                        f">>> [DEBUG] 왼팔 raw: {parsed[0:7]}  count={startup_count_left}"
                    )
                if startup_count_left >= STARTUP_WAIT_LEFT and any(
                    0 < parsed[i] < FLOATING_THRESHOLD for i in range(7)
                ):
                    system_ready_left = True
                for i in range(7):
                    if parsed[i] < FLOATING_THRESHOLD:
                        ema_values_left[i] = float(parsed[i])
                        if i == 6:
                            adc = int(ema_values_left[6])
                            ratio = max(
                                0.0,
                                min(
                                    1.0,
                                    (adc - GRIPPER_ADC_MIN_LEFT)
                                    / (GRIPPER_ADC_MAX_LEFT - GRIPPER_ADC_MIN_LEFT),
                                ),
                            )
                            prev_ticks_left[i] = int(
                                GRIPPER_POS_CLOSE_LEFT
                                + ratio
                                * (GRIPPER_POS_OPEN_LEFT - GRIPPER_POS_CLOSE_LEFT)
                            )
                        else:
                            init_tick = int(parsed[i])
                            if i in REVERSE_CHANNELS_LEFT:
                                init_tick = 4095 - init_tick
                            prev_ticks_left[i] = init_tick
                if system_ready_left:
                    print(">>> 왼팔 준비 완료")

            if not system_ready_right:
                for i in range(7):
                    ema_values_right[i] = float(parsed[i + 7])
                    if i == 6:
                        adc = int(ema_values_right[6])
                        ratio = max(
                            0.0,
                            min(
                                1.0,
                                (adc - GRIPPER_ADC_MIN_RIGHT)
                                / (GRIPPER_ADC_MAX_RIGHT - GRIPPER_ADC_MIN_RIGHT),
                            ),
                        )
                        prev_ticks_right[i] = int(
                            GRIPPER_POS_CLOSE_RIGHT
                            + ratio * (GRIPPER_POS_OPEN_RIGHT - GRIPPER_POS_CLOSE_RIGHT)
                        )
                    else:
                        init_tick = int(parsed[i + 7])
                        if i in REVERSE_CHANNELS_RIGHT:
                            init_tick = 4095 - init_tick
                        prev_ticks_right[i] = init_tick
                system_ready_right = True
                print(">>> 오른팔 준비 완료")

            time.sleep(0.02)
            continue

        if time.time() - last_serial_rx_time > SERIAL_TIMEOUT_SEC:
            if current_state_left != STATE_ERROR or current_state_right != STATE_ERROR:
                print(
                    f">>> [통신오류] {SERIAL_TIMEOUT_SEC}초 이상 ADC 데이터 없음 — 양팔 ERROR 전환"
                )
            current_state_left = STATE_ERROR
            current_state_right = STATE_ERROR

        process_left_arm()
        process_right_arm()

        lift_state = update_lift_state(parsed[16])

        # 팬틸트 값 계산만 (로컬 하드웨어 쓰기 없음 — pantilt_safe3.py도 함께 수정됨)
        update_pantilt(parsed, sw_toggle, PAN_ID, TILT_ID)
        pan_pos = pantilt_safe3.pan_pos
        tilt_pos = pantilt_safe3.tilt_pos

        print("\033[F", end="")

        left_raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        right_raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(
            f"L:{left_raw_str} | R:{right_raw_str}"
            + f" SW:{sw_toggle}"
            + f" PAN:{pantilt_safe3.pan_pos:4d}"
            + f" TILT:{pantilt_safe3.tilt_pos:4d}"
            + f" LIFT:{lift_state:+d}(raw:{parsed[16]:4d})"
            + f" L_STATE:{current_state_left}(prev:{prev_state_left}, cnt:{anomaly_count_left})"
            + f" R_STATE:{current_state_right}(prev:{prev_state_right}, cnt:{anomaly_count_right})"
        )

        # =====================================================
        # 라즈베리파이로 UDP 전송 — 라파 servo_receiver.py가
        # 기대하는 포맷 그대로: <L1~7,R9~15,Pan,Tilt,Lift> (17개 값)
        # =====================================================
        if all(t is not None for t in prev_ticks_left) and all(
            t is not None for t in prev_ticks_right
        ):
            udp_data = (
                "<"
                + ",".join(str(t) for t in prev_ticks_left)
                + ","
                + ",".join(str(t) for t in prev_ticks_right)
                + f",{pan_pos},{tilt_pos},{lift_state}>"
            )
            try:
                udp_sock.sendto(udp_data.encode("utf-8"), (RPI_IP, RPI_PORT))
            except Exception as e:
                print("UDP 전송 오류:", e)

        time.sleep(0.02)

except KeyboardInterrupt:
    pass

# =========================
# 종료 — 로컬 서보가 없으므로, 라파에 안전 자세 명령을 UDP로 한 번 더 보내고 끝냄
# =========================
running = False

NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]
NEUTRAL_PAN = 511
NEUTRAL_TILT = 511
NEUTRAL_LIFT = 0

shutdown_data = (
    "<"
    + ",".join(str(t) for t in NEUTRAL_TICKS_LEFT)
    + ","
    + ",".join(str(t) for t in NEUTRAL_TICKS_RIGHT)
    + f",{NEUTRAL_PAN},{NEUTRAL_TILT},{NEUTRAL_LIFT}>"
)
try:
    udp_sock.sendto(shutdown_data.encode("utf-8"), (RPI_IP, RPI_PORT))
    time.sleep(1.5)  # 라파 쪽 서보가 실제로 이동할 시간 확보
except Exception as e:
    print("종료 UDP 전송 오류:", e)

print("종료")
