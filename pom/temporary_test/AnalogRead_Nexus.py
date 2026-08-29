import serial
import threading
import struct
import time

# =====================================================
# [테스트 전용 파일] 서보 없이 가변저항 + STM32만으로
# Stage1/2 이상탐지 + 가속도 제한 + 통신 타임아웃 + CRC/시퀀스 자체테스트 검증
# =====================================================

STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

IDLE_CONFIRM_LOOPS = 10

# =====================================================
# Stage 1/2 이상탐지 파라미터
# =====================================================
ADC_MIN_VALID = 0
ADC_MAX_VALID = 4095

MAX_RAW_DELTA = 1500
EXTREME_LOW = 30
EXTREME_HIGH = 4065
ANOMALY_CONFIRM_COUNT = 3

prev_raw_left = [None] * 7
prev_raw_right = [None] * 7
anomaly_count_left = [0] * 7
anomaly_count_right = [0] * 7


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
    test_data = b"123456789"
    result = calc_crc16_ccitt(test_data)
    expected = 0x29B1
    status = "PASS" if result == expected else "FAIL"
    print(
        f">>> [CRC16 자체테스트] 계산값=0x{result:04X} 기대값=0x{expected:04X} → {status}"
    )


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
# [추가] 상하이동(리프트) 3구간 상태 판정 — 히스테리시스 적용
# 실측 기반 캘리브레이션:
#   - 전체 가동범위: 507(하강 끝) ~ 2483(상승 끝)
#   - 중립(가만히 둔 상태) 실측값: 1949~1977, 평균 1963
#   - 산술적 중앙값(1495)과 실제 중립값(1963)이 어긋나서
#     비율 계산 대신 중립 실측값 기준으로 임계값을 직접 잡음
#   - 히스테리시스 여유는 실제 테스트로 미세조정한 값
# =====================================================
LIFT_LOW_ENTER = 1400  # 중립(1963)에서 -563
LIFT_LOW_EXIT = 1600  # 중립(1963)에서 -363
LIFT_HIGH_ENTER = 2200  # 중립(1963)에서 +237
LIFT_HIGH_EXIT = 2000  # 중립(1963)에서 +37
LIFT_REVERSED = False  # 실측 후 반전이면 True로

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
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

parsed = [
    0
] * 17  # parsed[0:7]=왼팔, parsed[7:14]=오른팔, parsed[14]=VRx, parsed[15]=VRy, parsed[16]=상하이동

sw_toggle = 0
sw1_toggle_val = 0
running = True

FLOATING_THRESHOLD = 4080

SERIAL_TIMEOUT_SEC = 0.5
last_serial_rx_time = time.time()

# AdcPacket_t: header(2s)+msg_type(B)+seq_num(H)+mux_adc(16H)+adc_ind(5H)+sw0(B)+sw1(B)+crc(H)
PACKET_FORMAT = "<2sBH16H5HBBBH"  # H 앞에 B(key_states) 1개 추가
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 자동으로 52바이트로 계산됨

seq_validator_adc = SequenceValidator()


# [신규 추가] 1~5번 키캡 상태 전역 변수 선언 (Unused 회색 음영 방지)
key1_pressed = False
key2_pressed = False
key3_pressed = False
key4_pressed = False
key5_pressed = False


def read_serial_adc():
    global parsed, sw_toggle, sw1_toggle_val, running, last_serial_rx_time

    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    except Exception as e:
        print("시리얼 열기 실패:", e)
        return

    buf = bytearray()

    while running:
        try:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            buf.extend(data)

            while len(buf) >= PACKET_SIZE:
                idx = buf.find(b"\xaa\x55")
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PACKET_SIZE:
                    break

                packet_bytes = bytes(buf[:PACKET_SIZE])
                del buf[:PACKET_SIZE]

                unpacked = struct.unpack(PACKET_FORMAT, packet_bytes)
                seq_num = unpacked[2]
                mux_adc = unpacked[3:19]  # 16개
                adc_ind = unpacked[19:24]  # 5개 (PB1, PC0, PC1, PA4, PA5)

                sw0 = unpacked[24]
                sw1 = unpacked[25]
                key_states_raw = unpacked[26]  # [신규 추가] 1바이트 키 상태
                crc_recv = unpacked[27]  # [인덱스 수정] 기존 26에서 27로 한 칸 밀림

                crc_calc = calc_crc16_ccitt(packet_bytes[:-2])

                if crc_calc != crc_recv:
                    print(
                        f">>> [CRC 불일치] seq={seq_num} calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}"
                    )
                    continue

                key1_pressed = not bool(key_states_raw & (1 << 0))
                key2_pressed = not bool(key_states_raw & (1 << 1))
                key3_pressed = not bool(key_states_raw & (1 << 2))
                key4_pressed = not bool(key_states_raw & (1 << 3))
                key5_pressed = not bool(key_states_raw & (1 << 4))

                seq_result = seq_validator_adc.check(seq_num)
                if seq_result != "OK":
                    print(f">>> [SEQ {seq_result}] seq={seq_num}")

                # ===== 채널 매핑 (확인된 값 기준) =====
                for i in range(7):
                    parsed[i] = mux_adc[i + 1]  # 왼팔: mux_adc[1]~[7]
                for i in range(7):
                    parsed[i + 7] = mux_adc[i + 9]  # 오른팔: mux_adc[9]~[15]

                parsed[14] = adc_ind[3]  # PA4 (VRx)
                parsed[15] = adc_ind[2]  # PC1 (VRy)
                parsed[16] = adc_ind[4]  # PA5 (상하이동)  ← 새로 추가

                sw_toggle = sw0
                sw1_toggle_val = sw1

                last_serial_rx_time = time.time()

        except Exception as e:
            print("ERR:", e)


# =========================
# 왼팔 파라미터
# =========================
MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]

EMA_ALPHA_ARM_LEFT = [0.4, 0.4, 0.4, 0.4, 0.4, 0.5]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 15
DEAD_ZONE_EXIT_LEFT = 25
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
current_state_left = STATE_IDLE
prev_state_left = STATE_IDLE
idle_confirm_count_left = 0

system_ready_left = False
startup_count_left = 0
STARTUP_WAIT_LEFT = 80

# =========================
# 오른팔 파라미터
# =========================
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
REVERSE_CHANNELS_RIGHT = [0, 3, 4, 5, 6]

EMA_ALPHA_ARM_RIGHT = [0.45, 0.45, 0.45, 0.45, 0.45, 0.5]
EMA_ALPHA_GRIPPER_RIGHT = 0.5

DEAD_ZONE_ENTER_RIGHT = 12
DEAD_ZONE_EXIT_RIGHT = 20
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
current_state_right = STATE_IDLE
prev_state_right = STATE_IDLE
idle_confirm_count_right = 0

system_ready_right = False
startup_count_right = 0
STARTUP_WAIT_RIGHT = 50


def disable_left_torque():
    print(">>> ERROR: 왼팔 전 모터 토크 OFF (테스트 모드: 서보 없음)")


def process_left_arm():
    global ema_values_left, prev_ticks_left, in_dead_zone_left
    global current_state_left, idle_confirm_count_left, prev_state_left

    if check_anomaly(parsed[0:7], prev_raw_left, anomaly_count_left, "왼팔"):
        current_state_left = STATE_ERROR

    if current_state_left == STATE_ERROR:
        if prev_state_left != STATE_ERROR:
            disable_left_torque()
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
                if diff <= DEAD_ZONE_EXIT_LEFT:
                    continue
                else:
                    in_dead_zone_left[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_LEFT:
                    in_dead_zone_left[i] = True
                    continue

        prev_ticks_left[i] = tick


def disable_right_torque():
    print(">>> ERROR: 오른팔 전 모터 토크 OFF (테스트 모드: 서보 없음)")


def process_right_arm():
    global ema_values_right, prev_ticks_right, in_dead_zone_right
    global current_state_right, idle_confirm_count_right, prev_state_right

    if check_anomaly(parsed[7:14], prev_raw_right, anomaly_count_right, "오른팔"):
        current_state_right = STATE_ERROR

    if current_state_right == STATE_ERROR:
        if prev_state_right != STATE_ERROR:
            disable_right_torque()
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
                if diff <= DEAD_ZONE_EXIT_RIGHT:
                    continue
                else:
                    in_dead_zone_right[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_RIGHT:
                    in_dead_zone_right[i] = True
                    continue

        prev_ticks_right[i] = tick


t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

crc16_self_test()
sequence_validator_self_test()

print(
    "시작 (서보 없음 — Stage1/2 + 가속도 제한 + 통신 타임아웃 + 상하이동 판정 테스트)"
)

try:
    while True:

        if not (system_ready_left and system_ready_right):
            if not system_ready_left:
                startup_count_left += 1
                if startup_count_left >= STARTUP_WAIT_LEFT and any(
                    0 < parsed[i] < FLOATING_THRESHOLD for i in range(7)
                ):
                    system_ready_left = True
                    for i in range(7):
                        if parsed[i] < FLOATING_THRESHOLD:
                            ema_values_left[i] = float(parsed[i])
                    print(">>> 왼팔 준비 완료")

            if not system_ready_right:
                startup_count_right += 1
                if startup_count_right >= STARTUP_WAIT_RIGHT and any(
                    parsed[i + 7] > 0 for i in range(7)
                ):
                    system_ready_right = True
                    for i in range(7):
                        ema_values_right[i] = float(parsed[i + 7])
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

        lift_state = update_lift_state(parsed[16])  # ← 추가

        left_raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        right_raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(
            f"L:{left_raw_str} | R:{right_raw_str}"
            + f" SW:{sw_toggle}"
            + f" SW1:{sw1_toggle_val}"
            + f" LIFT:{lift_state:+d}(raw:{parsed[16]:4d})"
            + f" KEYS:[{int(key1_pressed)}{int(key2_pressed)}{int(key3_pressed)}{int(key4_pressed)}{int(key5_pressed)}]"
            + f" LJOY: PA4={parsed[14]:4d} PC1={parsed[15]:4d}"
            + f" L_STATE:{current_state_left}(prev:{prev_state_left}, cnt:{anomaly_count_left})"
            + f" R_STATE:{current_state_right}(prev:{prev_state_right}, cnt:{anomaly_count_right})"
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    pass

running = False
time.sleep(0.1)

print("종료")
