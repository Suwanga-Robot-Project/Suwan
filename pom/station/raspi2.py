import serial
import threading
import time
import struct  # CRC
import socket  # UDP 통신용
import pantilt_safe3
from pantilt_safe3 import update_pantilt

# ===== 그리퍼 자동교체 관련 모듈 (같은 폴더에 있어야 함) =====
import station_positions
import arm_swap_sequence
import key_input_handler

# ===== ADC 노이즈 필터 (adc_filter.py가 같은 폴더에 있어야 함) =====
from adc_filter import AdcFilter

adc_filter = AdcFilter()

# ===== 리프트(상하이동) 백엔드 — 라파의 lift_server.py에 원격 연결, 안 되면 가짜로 대체 =====
try:
    import lift_control_remote as lift_backend

    print(">>> [그리퍼교체] 원격 라파 연결(lift_control_remote) 사용 — 실제 하강/상승")
except Exception as e:
    import lift_control_sim as lift_backend

    print(
        f">>> [그리퍼교체] 라파 lift_server 연결 실패({e}) — 시뮬레이션(lift_control_sim)으로 대체"
    )

# =====================================================
# [PC용] 이 스크립트는 라즈베리파이의 servo_receiver.py와 짝을 이룹니다.
# PC는 STM32 ADC(COM13)만 로컬로 읽고, 연산(FSM/이상탐지/EMA/tick계산)을
# 전부 마친 뒤 최종 tick 값을 UDP로 라파에 보냅니다.
# 서보(왼팔/오른팔/팬틸트)는 전부 라파에 물려있어 여기서는 직접 제어하지 않습니다.
#
# ===== 그리퍼 자동교체 통합 (신규) =====
# 리미트 스위치(GPIO23/24)는 라파에만 물리적으로 존재하므로, 이 노트북에서는
# 직접 읽을 수 없습니다. 대신 lift_control_remote.py가 TCP로 라파의
# lift_server.py에 "내려가/올라가" 명령을 보내고, 라파가 실제 GPIO로 리미트
# 스위치를 감지해서 그 결과(걸린 시간)만 돌려받는 구조입니다.
# 팔 이동은 기존과 동일하게 UDP tick 전송 방식을 그대로 재사용하되, 그리퍼
# 교체 시퀀스 중에는 조이스틱이 아니라 계산된 스테이션 tick 값을 보냅니다.
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
PACKET_SIZE = 52
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBBH")  # sw1과 crc 사이 key_states(B) 추가

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
# ⚠️ 이건 "수동 조종용" 리프트 조이스틱 상태 표시이며, 그리퍼교체용
#    리미트스위치 기반 하강/상승(lift_backend)과는 별개입니다. 건드리지 않음.
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

adc_raw = [0] * 24
parsed = [
    0
] * 19  # [0:7]=왼팔, [7:14]=오른팔, [14:16]=오른쪽조이스틱IND, [16]=상하이동, [17]=왼쪽조이스틱ind2, [18]=왼쪽조이스틱ind3

sw_toggle = 0
sw1_toggle = 0
running = True

key1_pressed = False
key2_pressed = False
key3_pressed = False
key4_pressed = False
key5_pressed = False
key6_pressed = (
    False  # ← 신규 추가: 6번 키캡 (정상 semantics: 눌리면 True, 안 눌리면 False)
)
key7_pressed = False  # 7번 키캡 (정상 semantics: 눌리면 True, 안 눌리면 False)

FLOATING_THRESHOLD = 4080

SERIAL_TIMEOUT_SEC = 0.5
last_serial_rx_time = time.time()
first_packet_received = False


def read_serial_adc():
    global adc_raw, parsed, sw_toggle, sw1_toggle, running, last_serial_rx_time
    global key1_pressed, key2_pressed, key3_pressed, key4_pressed, key5_pressed, key6_pressed, key7_pressed
    global first_packet_received

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
                    key_states,
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
                mux_adc_vals = adc_filter.apply(mux_adc_vals)

                for i in range(16):
                    adc_raw[i] = mux_adc_vals[i]
                adc_raw[16] = ind0
                adc_raw[17] = ind1
                adc_raw[18] = ind2
                adc_raw[19] = ind3
                adc_raw[20] = ind4
                adc_raw[21] = sw0
                adc_raw[22] = sw1
                adc_raw[23] = key_states

                for i in range(7):
                    parsed[i] = adc_raw[i + 1]
                for i in range(7):
                    parsed[i + 7] = adc_raw[i + 9]
                parsed[14] = adc_raw[16]
                parsed[15] = adc_raw[17]
                parsed[16] = adc_raw[20]

                # 1. 조이스틱 원본 값 분리

                raw_x = adc_raw[18]
                raw_y = adc_raw[19]

                JOY_CENTER = 2033  # 조이스틱 입력 정중앙 값
                DEADZONE_RAW = 300  # 노이즈 방지용 데드존

                OUT_CENTER = 2033  # 모터 출력 중립(정지) 값
                OUT_RANGE = 2033  # 중립에서 최대/최소까지의 가변 범위 (0 ~ 4066 기준)

                # --------------------------------------------------
                # 1. X축 (회전) 속도 매핑
                # --------------------------------------------------
                if JOY_CENTER - DEADZONE_RAW < raw_x < JOY_CENTER + DEADZONE_RAW:
                    turn_val = OUT_CENTER  # 데드존 내에서는 2033으로 완벽 정지
                elif raw_x <= JOY_CENTER - DEADZONE_RAW:
                    # 0 ~ 2033 구간을 4066 ~ 2033으로 변환
                    turn_val = int(
                        OUT_CENTER + ((JOY_CENTER - raw_x) / JOY_CENTER) * OUT_RANGE
                    )
                else:
                    # 2033 ~ 4066 구간을 2033 ~ 0으로 변환
                    raw_x = min(raw_x, 4066)
                    turn_val = int(
                        OUT_CENTER
                        - ((raw_x - JOY_CENTER) / (4066 - JOY_CENTER)) * OUT_RANGE
                    )

                # --------------------------------------------------
                # 2. Y축 (전후진) 속도 매핑
                # --------------------------------------------------
                if JOY_CENTER - DEADZONE_RAW < raw_y < JOY_CENTER + DEADZONE_RAW:
                    throttle_val = OUT_CENTER  # 데드존 내에서는 2033으로 완벽 정지
                elif raw_y <= JOY_CENTER - DEADZONE_RAW:
                    # 0 ~ 2033 구간을 4066 ~ 2033으로 변환
                    throttle_val = int(
                        OUT_CENTER + ((JOY_CENTER - raw_y) / JOY_CENTER) * OUT_RANGE
                    )
                else:
                    # 2033 ~ 4066 구간을 2033 ~ 0으로 변환
                    raw_y = min(raw_y, 4066)
                    throttle_val = int(
                        OUT_CENTER
                        - ((raw_y - JOY_CENTER) / (4066 - JOY_CENTER)) * OUT_RANGE
                    )

                # 최종 패킷 대입
                parsed[17] = throttle_val
                parsed[18] = turn_val

                sw_toggle = adc_raw[21]
                sw1_toggle = adc_raw[22]

                # ⚠️ 요청에 따라 그대로 유지 (수정 안 함):
                # main.c에서 이미 반전 처리(bit=1이면 눌림)되어 있는데 여기서 not을
                # 또 붙여서 다시 뒤집고 있음 — 실제로는 "안 눌렸을 때 True"가 됨.
                key1_pressed = not bool(key_states & (1 << 0))
                key2_pressed = not bool(key_states & (1 << 1))
                key3_pressed = not bool(key_states & (1 << 2))
                key4_pressed = not bool(key_states & (1 << 3))
                key5_pressed = not bool(key_states & (1 << 4))
                # 6번 키(신규 추가) — 1~5번과 달리 반전 버그 없이 정상 처리
                key6_pressed = bool(key_states & (1 << 5))
                # 7번 키 — 마찬가지로 정상 처리
                # (main.c에서 bit=1이면 눌림 → 그대로 사용, not 안 붙임)
                key7_pressed = bool(key_states & (1 << 6))

                first_packet_received = True
                last_serial_rx_time = time.time()

                del buf[:PACKET_SIZE]

        except Exception as e:
            print("ERR:", e)


# =========================
# 왼팔 tick 계산 설정 (로컬 서보 없음 — UDP로 보낼 값만 계산)
# =========================
MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]

EMA_ALPHA_ARM_LEFT = [0.35, 0.35, 0.3, 0.3, 0.3, 0.3]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 28
DEAD_ZONE_EXIT_LEFT = 40
# ▼ 추가: 채널별 데드존 예외 {인덱스: (ENTER, EXIT)}
DEAD_ZONE_OVERRIDE_LEFT = {
    5: (70, 110),  # 왼팔 6번 임시 대응
}
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

EMA_ALPHA_ARM_RIGHT = [0.35, 0.35, 0.3, 0.3, 0.3, 0.3]
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
                dz_enter, dz_exit = DEAD_ZONE_OVERRIDE_LEFT.get(
                    i, (DEAD_ZONE_ENTER_LEFT, DEAD_ZONE_EXIT_LEFT)
                )
                diff = abs(tick - prev_ticks_left[i])
                if in_dead_zone_left[i]:
                    diff_from_anchor = abs(tick - dead_zone_anchor_left[i])
                    if diff_from_anchor <= dz_exit:
                        continue
                else:
                    in_dead_zone_left[i] = False
            else:
                if diff <= dz_enter:
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

        # ===== 니퍼 착용 중일 때만 그리퍼(7번 모터) 조종 범위 제한 =====
        # (2번 키로 니퍼 교체 후 원위치 복귀한 뒤 라이브 조종에 적용됨.
        #  다른 그리퍼로 바뀌면 다음 프레임부터 자동으로 해제됨)
        if i == 6 and GRIPPER_HELD_RIGHT == "nipper":
            try:
                tick = station_positions.clamp_for_nipper(tick)
            except ValueError:
                pass  # NIPPER_SAFE_TICK_RANGE 아직 미실측이면 clamp 없이 통과

        prev_ticks_right[i] = tick


# =====================================================
# ===== 그리퍼 자동교체 통합 (신규 추가, 2026-08-07 순서 재설계) =====
# =====================================================
GRIPPER_HELD_LEFT = None
GRIPPER_HELD_RIGHT = None
prev_key_edge_states = (False, False, False, False, False)

# ===== 타이밍 파라미터 =====
NEUTRAL_TRANSITION_SECONDS = 2.0  # 정자세로 "천천히" 이동하는데 걸리는 시간
NEUTRAL_TRANSITION_STEPS = 40  # 몇 단계로 나눠서 보간할지 (많을수록 부드러움)
NEUTRAL_WAIT_SECONDS = 2.0  # 정자세에서 대기하는 시간 (흔들림 방지)
STATION_APPROACH_SECONDS = 2.0  # 스테이션 위치로 "천천히" 이동하는데 걸리는 시간
STATION_APPROACH_STEPS = 40


def move_arm_to(arm_side, ticks):
    """
    그리퍼 자동교체 시퀀스가 각 단계마다 호출하는 함수.
    실제 서보는 라파에 물려있으므로, prev_ticks_left/right를 갱신하고
    그 자리에서 즉시 UDP로 라파에 전송함 (메인 루프의 다음 차례를 기다리지 않음 —
    각 단계별 sleep()이 그 프레임의 지속시간 역할을 함).
    """
    global prev_ticks_left, prev_ticks_right

    target_array = prev_ticks_left if arm_side == "left" else prev_ticks_right
    for i, t in enumerate(ticks):
        if t is not None:
            target_array[i] = int(t)

    if not (
        all(t is not None for t in prev_ticks_left)
        and all(t is not None for t in prev_ticks_right)
    ):
        return  # 아직 양팔 초기화 전이면 전송 스킵

    key_states_str = f"{int(key1_pressed)}{int(key2_pressed)}{int(key3_pressed)}{int(key4_pressed)}{int(key5_pressed)}{int(key6_pressed)}{int(key7_pressed)}"
    udp_data = (
        "<"
        + ",".join(str(t) for t in prev_ticks_left)
        + ","
        + ",".join(str(t) for t in prev_ticks_right)
        + f",{pantilt_safe3.pan_pos},{pantilt_safe3.tilt_pos},{lift_state}"
        + f",{sw1_toggle},{parsed[17]},{parsed[18]}"
        + f",{key_states_str}"
        + f",{sw_toggle}>"  # ← 신규: 오른쪽 노브(SW2, 팬/틸트용) 클릭 상태, 인덱스 21
    )
    try:
        udp_sock.sendto(udp_data.encode("utf-8"), (RPI_IP, RPI_PORT))
    except Exception as e:
        print("UDP 전송 오류(그리퍼교체):", e)


def _lerp_ticks(start_ticks, end_ticks, ratio):
    """start_ticks에서 end_ticks까지 ratio(0.0~1.0) 비율만큼 보간된 tick 리스트."""
    result = []
    for s, e in zip(start_ticks, end_ticks):
        if s is None or e is None:
            result.append(e)
        else:
            result.append(int(round(s + (e - s) * ratio)))
    return result


# ===== UDP 유실 대비 — 이동 끝난 뒤 최종값 재전송 =====
FINAL_RESEND_COUNT = 4  # 몇 번 더 보낼지
FINAL_RESEND_DELAY = 0.05  # 재전송 간격(초)


def _resend_final(arm_side, final_ticks):
    """
    UDP는 도착 보장이 없어서, 특히 '마지막 목표값' 패킷이 유실되면
    라파가 그 전 중간값에서 멈춰버릴 수 있음. 이동이 끝난 뒤 최종값을
    몇 번 더 반복 전송해서, 적어도 하나는 도착하도록 함.
    """
    for _ in range(FINAL_RESEND_COUNT):
        move_arm_to(arm_side, final_ticks)
        time.sleep(FINAL_RESEND_DELAY)


def move_arm_gradually(arm_side, target_ticks, duration_seconds, steps=40):
    """
    (기존 방식, 지금은 안 씀 — 참고용으로 남겨둠)
    현재 위치에서 target_ticks까지 7개 모터 전부 동시에 보간 이동.
    """
    current = list(prev_ticks_left if arm_side == "left" else prev_ticks_right)
    step_delay = duration_seconds / steps
    for step in range(1, steps + 1):
        ratio = step / steps
        interp = _lerp_ticks(current, target_ticks, ratio)
        move_arm_to(arm_side, interp)
        time.sleep(step_delay)
    _resend_final(arm_side, target_ticks)  # ← UDP 유실 대비 마무리 재전송


# ===== 모터별 순차이동 파라미터 =====
MOTOR_BY_MOTOR_DURATION = 0.3  # 모터 하나 이동에 걸리는 시간(초)
MOTOR_BY_MOTOR_STEPS = 10  # 모터 하나 이동을 몇 단계로 나눌지


def move_arm_motor_by_motor(
    arm_side,
    target_ticks,
    duration_per_motor=MOTOR_BY_MOTOR_DURATION,
    steps_per_motor=MOTOR_BY_MOTOR_STEPS,
):
    """
    현재 위치에서 target_ticks까지, 모터를 1번부터 7번까지 순서대로 하나씩 이동.
    (7개가 동시에 안 움직이고, 1번 모터가 목표에 다 도달한 뒤 2번 모터가 움직이기
    시작하는 식 — 한 모터씩 딱딱 움직이는 걸 눈으로 확인하고 싶을 때 사용)
    각 모터 자체의 이동은 보간으로 부드럽게 처리됨.
    이미 목표와 같은 모터는 건너뜀(시간 낭비 방지).
    """
    working = list(prev_ticks_left if arm_side == "left" else prev_ticks_right)
    for idx in range(len(working)):
        if target_ticks[idx] is None or working[idx] is None:
            continue
        start_tick = working[idx]
        target_tick = target_ticks[idx]
        if start_tick == target_tick:
            continue
        step_delay = duration_per_motor / steps_per_motor
        for step in range(1, steps_per_motor + 1):
            ratio = step / steps_per_motor
            working[idx] = int(round(start_tick + (target_tick - start_tick) * ratio))
            move_arm_to(arm_side, working)
            time.sleep(step_delay)
        working[idx] = target_tick
    _resend_final(arm_side, working)  # ← UDP 유실 대비 마무리 재전송 (전체 7개 최종값)


def run_gripper_swap(arm_side, target_gripper):
    """
    한 팔의 그리퍼 교체 전체 시퀀스 (블로킹 — 끝날 때까지 메인 루프가 멈춤).
    A안 규칙: 이게 실행되는 동안 메인 루프 자체가 진행 안 되므로,
    자연스럽게 "IDLE 아닐 때 새 키 입력 무시"가 구현됨(별도 상태 체크 불필요).

    ===== 순서 (2026-08-07 재설계) =====
    공통: 원래 위치 저장 → NEUTRAL로 천천히 이동 → 4초 대기(흔들림 방지)

    [빈손일 때]
      → 목표 스테이션 위치로 천천히 이동(아직 위, 안 내려감)
      → 하강(리미트스위치까지) → 부착(열린 상태로 접근→조이기)
      → 상승 → 원래 위치로 복귀

    [이미 그리퍼를 들고 있을 때]
      → 보유 중인 그리퍼의 스테이션 위치로 천천히 이동(아직 위)
      → 하강 → 탈거(최대조임→최대개방)
      → B안 클리어런스 순차이동(기존 실측값 그대로: 오른팔은 모터1→모터4,
        왼팔은 모터4→모터1 순서로 살짝 든 채 옆 스테이션 방향으로 이동)
      → 목표 스테이션 위치로 이동 → 부착
      → 상승 → 원래 위치로 복귀
    """
    global GRIPPER_HELD_LEFT, GRIPPER_HELD_RIGHT

    other_side = "right" if arm_side == "left" else "left"

    held = GRIPPER_HELD_LEFT if arm_side == "left" else GRIPPER_HELD_RIGHT
    saved_ticks = list(prev_ticks_left if arm_side == "left" else prev_ticks_right)
    other_saved_ticks = list(
        prev_ticks_right if arm_side == "left" else prev_ticks_left
    )
    neutral_ticks = (
        station_positions.NEUTRAL_TICKS_LEFT
        if arm_side == "left"
        else station_positions.NEUTRAL_TICKS_RIGHT
    )

    print(f"\n>>> [{arm_side} 팔] 그리퍼교체 시작: {held} → {target_gripper}")
    print(f"    (원래 위치 저장: {saved_ticks})")

    # ===== 공통: NEUTRAL로 천천히 이동 → 4초 대기 =====
    print(f"    → NEUTRAL(정자세)로 천천히 이동")
    move_arm_gradually(
        arm_side, neutral_ticks, NEUTRAL_TRANSITION_SECONDS, NEUTRAL_TRANSITION_STEPS
    )
    print(f"    → {NEUTRAL_WAIT_SECONDS}초 대기 (흔들림 방지)")
    time.sleep(NEUTRAL_WAIT_SECONDS)

    # ===== 반대편 팔을 안전 자세로 파킹 (충돌 방지) =====
    safe_ticks = station_positions.get_safe_retreat_ticks(other_side)
    if safe_ticks is not None:
        print(f"    → 반대편({other_side}) 팔을 안전 자세로 파킹")
        move_arm_motor_by_motor(other_side, safe_ticks)
    else:
        print(
            f"    [경고] {other_side} 안전자세(SAFE_RETREAT_MOTOR1) 미실측 — 파킹 건너뜀, 충돌 위험 있음"
        )

    if held is None:
        # ===== 빈손: 목표 스테이션 위치로 이동(아직 위) → 하강 → 부착 =====
        target_ticks = station_positions.get_corrected_station_ticks(
            arm_side, target_gripper
        )
        if target_ticks is None:
            raise ValueError(
                f"{arm_side}/{target_gripper} tick 값이 아직 실측되지 않았습니다"
            )

        print(f"    → 목표({target_gripper}) 스테이션 위치로 천천히 이동 (아직 위)")
        move_arm_motor_by_motor(arm_side, target_ticks)

        elapsed = lift_backend.descend_until_bottom_switch()
        time.sleep(1.0)  # 흔들림 안정화

        arm_swap_sequence._attach_at(
            arm_side, target_gripper, target_ticks, move_arm_to
        )
        new_held = target_gripper

    else:
        # ===== 보유 중: 보유 스테이션 위치로 이동(아직 위) → 하강 → 탈거
        #      → (target_gripper가 있으면) B안 클리어런스 → 목표 부착
        #      → (target_gripper가 None이면 = 5번 전체탈거) 탈거만 하고 끝 =====
        held_ticks = station_positions.get_corrected_station_ticks(arm_side, held)
        if held_ticks is None:
            raise ValueError(f"{arm_side}/{held} tick 값이 아직 실측되지 않았습니다")

        print(f"    → 보유 중인({held}) 스테이션 위치로 천천히 이동 (아직 위)")
        move_arm_motor_by_motor(arm_side, held_ticks)

        elapsed = lift_backend.descend_until_bottom_switch()
        time.sleep(1.0)  # 흔들림 안정화

        after_detach_ticks = arm_swap_sequence._detach_at(
            arm_side, held, held_ticks, move_arm_to
        )

        if target_gripper is None:
            # ===== 5번(전체탈거): 새로 부착할 대상 없음 — 탈거만 하고 끝 =====
            print(f"    → 전체탈거: 새 그리퍼 없이 빈손으로 완료")
            new_held = None
        else:
            clearance_seq = station_positions.get_direct_swap_clearance(
                arm_side, held, target_gripper
            )
            if clearance_seq:
                print(f"    → B안 클리어런스 순차이동")
                after_detach_ticks = arm_swap_sequence._move_sequential(
                    arm_side, after_detach_ticks, clearance_seq, move_arm_to
                )
            else:
                print(f"    [경고] {held}->{target_gripper} 클리어런스 미실측 — 건너뜀")

            target_ticks = station_positions.get_corrected_station_ticks(
                arm_side, target_gripper
            )
            if target_ticks is None:
                raise ValueError(
                    f"{arm_side}/{target_gripper} tick 값이 아직 실측되지 않았습니다"
                )

            arm_swap_sequence._attach_at(
                arm_side, target_gripper, target_ticks, move_arm_to
            )
            new_held = target_gripper

    lift_backend.ascend_full(elapsed)

    # ===== NEUTRAL을 경유해서 원래 위치로 복귀 =====
    # (스테이션 위치 → 원래 위치로 바로 가면, 로봇 구조상 상하이동 레일과
    #  겹치는 경로를 지나갈 수 있어서 엉킴/충돌 위험이 있음. 나갈 때(원래위치→
    #  스테이션)는 NEUTRAL을 거치는데 돌아올 때만 안 거쳐서 생긴 비대칭 문제.
    #  같은 안전 경로를 그대로 역순으로 써서 대칭을 맞춤.)
    print(f"    → NEUTRAL 경유해서 복귀 (충돌 방지)")
    move_arm_motor_by_motor(arm_side, neutral_ticks)

    print(f"    → 원래 위치로 천천히 복귀")
    move_arm_motor_by_motor(arm_side, saved_ticks)

    # ===== 반대편 팔도 원래 위치로 복귀 =====
    if safe_ticks is not None:
        print(f"    → 반대편({other_side}) 팔도 원래 위치로 복귀")
        move_arm_motor_by_motor(other_side, other_saved_ticks)

    if arm_side == "left":
        GRIPPER_HELD_LEFT = new_held
    else:
        GRIPPER_HELD_RIGHT = new_held

    print(f">>> [{arm_side} 팔] 그리퍼교체 완료, 새 상태: {new_held}\n")


def check_gripper_swap_trigger():
    """
    키캡 엣지(새로 눌린 순간)를 감지해서 그리퍼교체를 트리거.
    True를 반환하면 이번 루프에서 스왑이 실행됐다는 뜻 (호출부에서 continue 처리 필요).

    ⚠️ key1_pressed~key5_pressed는 현재 반전된 상태(안 눌렸을 때 True)로 남아있음
       (요청에 따라 수정하지 않음). 그 결과 여기서의 엣지 감지는 "새로 눌린 순간"이
       아니라 "누르고 있다가 손을 뗀 순간"에 반응하게 됩니다 — 실제 버튼을 누를 때가
       아니라 뗄 때 교체가 시작될 수 있습니다.
    """
    global prev_key_edge_states, GRIPPER_HELD_LEFT, GRIPPER_HELD_RIGHT

    current_keys = (
        key1_pressed,
        key2_pressed,
        key3_pressed,
        key4_pressed,
        key5_pressed,
    )
    edge_keys = tuple(
        now and not prev for now, prev in zip(current_keys, prev_key_edge_states)
    )
    prev_key_edge_states = current_keys

    if not any(edge_keys):
        return False

    e1, e2, e3, e4, e5 = edge_keys
    left_target, right_target = key_input_handler.parse_key_input(e1, e2, e3, e4, e5)

    triggered = False

    if left_target is not None:
        actual = None if left_target == key_input_handler.DROP_ALL else left_target
        if actual != GRIPPER_HELD_LEFT:
            run_gripper_swap("left", actual)
            triggered = True

    if right_target is not None:
        actual = None if right_target == key_input_handler.DROP_ALL else right_target
        if actual != GRIPPER_HELD_RIGHT:
            run_gripper_swap("right", actual)
            triggered = True

    return triggered


# =========================
# ADC thread (PC 로컬, 1개만 실행)
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

crc16_self_test()
sequence_validator_self_test()

adc_filter.reset()
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
                startup_count_right += 1
                if startup_count_right >= STARTUP_WAIT_RIGHT and any(
                    0 < parsed[i + 7] < FLOATING_THRESHOLD for i in range(7)
                ):
                    system_ready_right = True
                for i in range(7):
                    if parsed[i + 7] < FLOATING_THRESHOLD:
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
                                + ratio
                                * (GRIPPER_POS_OPEN_RIGHT - GRIPPER_POS_CLOSE_RIGHT)
                            )
                        else:
                            init_tick = int(parsed[i + 7])
                            if i in REVERSE_CHANNELS_RIGHT:
                                init_tick = 4095 - init_tick
                            prev_ticks_right[i] = init_tick
                if system_ready_right:
                    print(">>> 오른팔 준비 완료")

            time.sleep(0.02)
            continue

        if (
            first_packet_received
            and time.time() - last_serial_rx_time > SERIAL_TIMEOUT_SEC
        ):
            if current_state_left != STATE_ERROR or current_state_right != STATE_ERROR:
                print(
                    f">>> [통신오류] {SERIAL_TIMEOUT_SEC}초 이상 ADC 데이터 없음 — 양팔 ERROR 전환"
                )
            current_state_left = STATE_ERROR
            current_state_right = STATE_ERROR

        # ===== 그리퍼 자동교체 트리거 체크 (신규) =====
        # 여기서 실제로 교체가 실행되면(블로킹) 그동안 조이스틱 입력은 자동으로
        # 무시됨 — 아래 process_left_arm()/process_right_arm()이 이번 루프에서는
        # 실행되지 않고 건너뛰기 때문.
        if check_gripper_swap_trigger():
            time.sleep(0.02)
            continue

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
            f"----------------------------------------------\n"
            f"[팔]  L:{left_raw_str}\n"
            f"      R:{right_raw_str}\n"
            f"[상태] L_STATE:{current_state_left:5s}(prev:{prev_state_left:5s})  "
            f"R_STATE:{current_state_right:5s}(prev:{prev_state_right:5s})\n"
            f"[팬틸트] SW:{sw_toggle}  PAN:{pantilt_safe3.pan_pos:4d}  TILT:{pantilt_safe3.tilt_pos:4d}\n"
            f"[상하이동] LIFT:{lift_state:+d} (raw:{parsed[16]:4d})\n"
            f"[바퀴조이스틱] SW1:{sw1_toggle}  ind2:{parsed[17]:4d}  ind3:{parsed[18]:4d}\n"
            f"[키캡] [{int(key1_pressed)}{int(key2_pressed)}{int(key3_pressed)}{int(key4_pressed)}{int(key5_pressed)}{int(key6_pressed)}{int(key7_pressed)}]\n"
            f"[그리퍼] LEFT:{GRIPPER_HELD_LEFT}  RIGHT:{GRIPPER_HELD_RIGHT}\n"
            f"----------------------------------------------"
        )

        # =====================================================
        # 라즈베리파이로 UDP 전송 — 라파 servo_receiver.py가
        # 기대하는 포맷 그대로: <L1~7,R9~15,Pan,Tilt,Lift> (17개 값)
        # =====================================================
        if all(t is not None for t in prev_ticks_left) and all(
            t is not None for t in prev_ticks_right
        ):
            key_states_str = f"{int(key1_pressed)}{int(key2_pressed)}{int(key3_pressed)}{int(key4_pressed)}{int(key5_pressed)}{int(key6_pressed)}{int(key7_pressed)}"

            udp_data = (
                "<"
                + ",".join(str(t) for t in prev_ticks_left)
                + ","
                + ",".join(str(t) for t in prev_ticks_right)
                + f",{pan_pos},{tilt_pos},{lift_state}"
                + f",{sw1_toggle},{parsed[17]},{parsed[18]}"
                + f",{key_states_str}"
                + f",{sw_toggle}>"  # ← 신규: 오른쪽 노브(SW2, 팬/틸트용) 클릭 상태, 인덱스 21
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
    + f",{NEUTRAL_PAN},{NEUTRAL_TILT},{NEUTRAL_LIFT}"
    + f",0,0,0"
    + f",00000>"
)
try:
    udp_sock.sendto(shutdown_data.encode("utf-8"), (RPI_IP, RPI_PORT))
    time.sleep(1.5)  # 라파 쪽 서보가 실제로 이동할 시간 확보
except Exception as e:
    print("종료 UDP 전송 오류:", e)

adc_filter.report()
print("종료")
