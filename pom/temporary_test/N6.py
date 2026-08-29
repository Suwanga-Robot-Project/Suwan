import pantilt_safe2
import serial
import threading
import time
import struct  # CRC
import socket  # [추가] UDP 통신용
from scservo_sdk import *
from pantilt_safe2 import update_pantilt
from pantilt_safe2 import scs_write_pos
from pantilt_safe2 import pan_pos
from pantilt_safe2 import tilt_pos

# PC에서 라파 없이 알고리즘만 테스트할 때
from whells_safeN import update_wheels
from adc_filter import AdcFilter

adc_filter = AdcFilter()
# =====================================================
# [공통] FSM 상태 정의
# IDLE  : 해당 팔 전 채널 정지 상태, 모터 명령 안 보냄
# MOVE  : 해당 팔 최소 1채널 이상 움직이는 정상 동작 상태
# ERROR : 이상 감지 시 진입, 해당 팔 모든 모터 정지 및 명령 차단
# 왼팔/오른팔은 서로 독립된 FSM 상태를 가짐
# =====================================================
STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

IDLE_CONFIRM_LOOPS = 10  # 약 0.2초(루프 0.02s 기준), 채터링 방지용 debounce

# =====================================================
# [추가] Stage 1/2 이상탐지 파라미터
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
# [추가] CRC-16 CCITT (직접 구현)
# 다항식: 0x1021, 초기값: 0xFFFF (CCITT-FALSE 변형)
# 함수 정확성 확인용 자체 테스트 벡터만 아래 self_test로 검증.
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
    이 값과 일치하면 구현이 맞다는 뜻.
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

        # [수정] 실제로 전진한 경우(OK, LOST)에만 last_seq 갱신.
        # DUPLICATE/REORDER는 "이미 지나간 옛 패킷이 늦게 도착한 것"이라
        # 최신 위치 기준점을 되돌리면 안 됨 (그러면 다음 패킷까지 오판됨)
        if result in ("OK", "LOST"):
            self.last_seq = seq

        return result

    # =================================================

    def _is_ahead(self, seq, expected):
        # expected보다 seq가 앞서 있으면(순환 고려) 중간 패킷이 유실된 것
        diff = (seq - expected) % (self.max_seq + 1)
        return diff < (self.max_seq // 2)


# =====================================================
# =====================================================
# [추가] 실전 시퀀스 검증기 인스턴스 (Step2)
# =====================================================
PACKET_HEADER = b"\xaa\x55"
PACKET_SIZE = 52
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBBH")  # sw1과 crc 사이에 B(key_states) 추가

# [신규 추가] 키 상태 전역 변수 선언 (이렇게 해야 밖에서도 쓸 수 있음)
key1_pressed = False
key2_pressed = False
key3_pressed = False
key4_pressed = False
key5_pressed = False

seq_checker = SequenceValidator()


def sequence_validator_self_test():
    """
    가짜 시퀀스로 유실/중복/순서뒤바뀜 각각 정상 감지되는지 확인
    """
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


# --------------------=====================


def check_anomaly(channel_parsed, prev_raw, anomaly_count, arm_name):
    """
    Stage 1: 절대범위(0~4095) 이탈 → 즉시 이상 판정
    Stage 2: 변화율 급변 또는 극단값 왕복(단선 패턴) → 채널별 연속 카운트,
            ANOMALY_CONFIRM_COUNT(3회) 연속되면 True(ERROR) 리턴
    """
    error_triggered = False

    for i in range(7):
        raw = channel_parsed[i]

        # ---------- Stage 1: 절대범위 검증 ----------
        if raw < ADC_MIN_VALID or raw > ADC_MAX_VALID:
            print(f">>> [Stage1] {arm_name} 채널{i} 범위 이탈 raw={raw}")
            error_triggered = True
            continue  # 범위 자체가 깨졌으니 Stage2 비교 의미 없음

        # ---------- Stage 2: 변화율 + 단선 패턴 검증 ----------
        is_anomaly_frame = False

        if prev_raw[i] is not None:
            delta = abs(raw - prev_raw[i])

            if delta > MAX_RAW_DELTA:
                is_anomaly_frame = True

            # [수정] "극단값 안에서 미세하게 다름"이 아니라
            # "LOW 쪽에 있다가 HIGH 쪽으로, 또는 그 반대로 튀는지"만 판정
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
# STM32 ADC 시리얼 (양팔 공용, 스레드 1개만 실행)
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

# =====================================================
# [추가] 라즈베리파이 UDP 전송 설정
# =====================================================
RPI_IP = "192.168.0.24"
RPI_PORT = 5005
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =============================================

adc_raw = [0] * 23
parsed = [
    0
] * 19  # [0:7]=왼팔, [7:14]=오른팔, [14:16]=오른쪽조이스틱IND, [16]=상하이동, [17]=왼쪽조이스틱ind2, [18]=왼쪽조이스틱ind3

sw_toggle = 0
sw1_toggle = 0
running = True

FLOATING_THRESHOLD = 4080

# =====================================================
# [추가] 통신 타임아웃 감지
# =====================================================
SERIAL_TIMEOUT_SEC = 0.5  # 이 시간 이상 새 데이터 없으면 통신 두절로 판단
last_serial_rx_time = time.time()
first_packet_received = False  # ← 추가

# =========================================
# CRC 펌웨어 전환으로 바꾼 코드
# ==========================================


def read_serial_adc():
    global adc_raw, parsed, sw_toggle, sw1_toggle, running, last_serial_rx_time
    global key1_pressed, key2_pressed, key3_pressed, key4_pressed, key5_pressed
    global first_packet_received  # ← 추가
    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    except Exception as e:
        print("시리얼 열기 실패:", e)
        return

    buf = bytearray()  # [추가] 수신된 바이트를 계속 쌓아두는 버퍼

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
                    key_states_raw,  # [신규 추가] 1바이트 키 상태 비트맵
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
                adc_raw[20] = ind4  # ← 상하이동 원시값, 새로 추가
                adc_raw[21] = sw0
                adc_raw[22] = sw1

                for i in range(7):
                    parsed[i] = adc_raw[i + 1]
                for i in range(7):
                    parsed[i + 7] = adc_raw[i + 9]
                parsed[14] = adc_raw[16]
                parsed[15] = adc_raw[17]
                parsed[16] = adc_raw[20]  # ← 상하이동 원시값
                parsed[17] = adc_raw[18]  # ← 왼쪽 조이스틱 ind2
                parsed[18] = adc_raw[19]  # ← 왼쪽 조이스틱 ind3
                sw_toggle = adc_raw[21]  # ← 20에서 21로 인덱스 이동
                sw1_toggle = adc_raw[22]  # ← 왼쪽 조이스틱 전용 스위치

                key1_pressed = not bool(key_states_raw & (1 << 0))
                key2_pressed = not bool(key_states_raw & (1 << 1))
                key3_pressed = not bool(key_states_raw & (1 << 2))
                key4_pressed = not bool(key_states_raw & (1 << 3))
                key5_pressed = not bool(key_states_raw & (1 << 4))

                first_packet_received = True  # ← 추가
                last_serial_rx_time = time.time()

                del buf[:PACKET_SIZE]

        except Exception as e:
            print("ERR:", e)


# 이후 기존 EMA / 데드존 / tick 매핑은 그대로


# =========================
# 왼팔 STS3215 설정
# =========================
DEVICENAME_LEFT = "COM12"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]  # 모터 ID 6 반전

EMA_ALPHA_ARM_LEFT = [0.35, 0.35, 0.35, 0.3, 0.4, 0.3]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 28
DEAD_ZONE_EXIT_LEFT = 40
# ▼ 추가: 채널별 데드존 예외 {인덱스: (ENTER, EXIT)}
DEAD_ZONE_OVERRIDE_LEFT = {
    5: (70, 110),  # 왼팔 6번 임시 대응
}
MAX_DELTA_LEFT = 70
MAX_ACCEL_LEFT = 15  # [추가] 한 루프당 delta 변화량(가속도) 제한

GRIPPER_ADC_MIN_LEFT = 145
GRIPPER_ADC_MAX_LEFT = 1270
GRIPPER_POS_OPEN_LEFT = 4100
GRIPPER_POS_CLOSE_LEFT = 500

ema_values_left = [None] * 7
prev_ticks_left = [None] * 7
prev_delta_left = [0] * 7  # [추가] 직전 루프의 실제 delta 기억 (가속도 계산용)
in_dead_zone_left = [False] * 7
dead_zone_anchor_left = [None] * 7  # [추가] 데드존 진입 시점 기준 위치
current_state_left = STATE_IDLE
prev_state_left = STATE_IDLE  # [추가] ERROR 진입 순간 판단용
idle_confirm_count_left = 0

system_ready_left = False
startup_count_left = 0
STARTUP_WAIT_LEFT = 80

# =========================
# 오른팔 + 팬틸트 STS3215 설정
# =========================
DEVICENAME_RIGHT = "COM14"

MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
REVERSE_CHANNELS_RIGHT = [0, 3, 4, 5, 6]  # 9, 12, 13, 14, 15번 반전

PAN_ID = 22
TILT_ID = 33

EMA_ALPHA_ARM_RIGHT = [0.35, 0.35, 0.35, 0.3, 0.4, 0.7]
EMA_ALPHA_GRIPPER_RIGHT = 0.5

DEAD_ZONE_ENTER_RIGHT = 28
DEAD_ZONE_EXIT_RIGHT = 40
MAX_DELTA_RIGHT = 70
MAX_ACCEL_RIGHT = 15  # [추가]

GRIPPER_ADC_MIN_RIGHT = 2973
GRIPPER_ADC_MAX_RIGHT = 3993
GRIPPER_POS_OPEN_RIGHT = 3935
GRIPPER_POS_CLOSE_RIGHT = 0

ema_values_right = [None] * 7
prev_ticks_right = [None] * 7
prev_delta_right = [0] * 7  # [추가]
in_dead_zone_right = [False] * 7
dead_zone_anchor_right = [None] * 7  # [추가] 데드존 진입 시점 기준 위치
current_state_right = STATE_IDLE
prev_state_right = STATE_IDLE  # [추가]
idle_confirm_count_right = 0

system_ready_right = False
startup_count_right = 0
STARTUP_WAIT_RIGHT = 50


# =========================
# [추가] 왼팔 ERROR 시 토크 OFF
# =========================
def disable_left_torque():
    for m in MOTORS_LEFT:
        packetHandler_left.write1ByteTxRx(
            portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
    print(">>> ERROR: 왼팔 전 모터 토크 OFF 완료")


# =========================
# 왼팔 처리 함수
# ERROR 상태일 때 모터 토크 OFF
# =========================
def process_left_arm(portHandler, packetHandler):
    global ema_values_left, prev_ticks_left, in_dead_zone_left
    global current_state_left, idle_confirm_count_left, prev_state_left
    global dead_zone_anchor_left

    # 호출부 수정 2단계 이상값 탐지 구현
    if check_anomaly(parsed[0:7], prev_raw_left, anomaly_count_left, "왼팔"):
        current_state_left = STATE_ERROR

    if current_state_left == STATE_ERROR:
        # [추가] ERROR 진입 순간(직전 상태가 ERROR가 아니었을 때)에만 토크 OFF
        if prev_state_left != STATE_ERROR:
            disable_left_torque()
        prev_state_left = current_state_left
        return

    prev_state_left = current_state_left  # [추가] 정상 루프에서도 상태 기록

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
        m = MOTORS_LEFT[i]
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

            # =====================================================
            # [추가] 급가속/급정거 방지 — 가속도(delta 변화량) 제한
            # 속도 자체가 아니라 "속도가 얼마나 빨리 바뀌는가"를 제한
            # =====================================================
            actual_delta = tick - prev_ticks_left[i]
            accel = actual_delta - prev_delta_left[i]
            if accel > MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] + MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            elif accel < -MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] - MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            prev_delta_left[i] = actual_delta

            # =============================================

            if i != 6:
                if i in REVERSE_CHANNELS_LEFT:
                    ema_values_left[i] = float(4095 - tick)
                else:
                    ema_values_left[i] = float(tick)

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

        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)
        prev_ticks_left[i] = tick


# =========================
# [추가] 오른팔 ERROR 시 토크 OFF
# =========================
def disable_right_torque():
    for m in MOTORS_RIGHT:
        packetHandler_right.write1ByteTxRx(
            portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
    print(">>> ERROR: 오른팔 전 모터 토크 OFF 완료")


# =========================
# 오른팔 처리 함수
# ERROR 상태일 때 모터 토크 OFF
# =========================
def process_right_arm(portHandler, packetHandler):
    global ema_values_right, prev_ticks_right, in_dead_zone_right
    global current_state_right, idle_confirm_count_right, prev_state_right
    global dead_zone_anchor_right

    if check_anomaly(parsed[7:14], prev_raw_right, anomaly_count_right, "오른팔"):
        current_state_right = STATE_ERROR

    if current_state_right == STATE_ERROR:
        # [추가] ERROR 진입 순간에만 토크 OFF
        if prev_state_right != STATE_ERROR:
            disable_right_torque()
        prev_state_right = current_state_right
        return

    prev_state_right = current_state_right  # [추가]

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
        m = MOTORS_RIGHT[i]
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

            # =====================================================
            # [추가] 급가속/급정거 방지 — 가속도(delta 변화량) 제한
            # 속도 자체가 아니라 "속도가 얼마나 빨리 바뀌는가"를 제한
            # (왼팔 process_left_arm과 동일한 로직)
            # =====================================================
            actual_delta = tick - prev_ticks_right[i]
            accel = actual_delta - prev_delta_right[i]
            if accel > MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] + MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            elif accel < -MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] - MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            prev_delta_right[i] = actual_delta

            # =============================================

            if i != 6:
                if i in REVERSE_CHANNELS_RIGHT:
                    ema_values_right[i] = float(4095 - tick)
                else:
                    ema_values_right[i] = float(tick)

            diff = abs(tick - prev_ticks_right[i])
            if in_dead_zone_right[i]:
                # [수정] 직전 프레임이 아니라 "고정된 기준점"과의 차이로 판정
                diff_from_anchor = abs(tick - dead_zone_anchor_right[i])
                if diff_from_anchor <= DEAD_ZONE_EXIT_RIGHT:
                    continue
                else:
                    in_dead_zone_right[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_RIGHT:
                    in_dead_zone_right[i] = True
                    dead_zone_anchor_right[i] = (
                        tick  # [추가] 이 순간 위치를 기준점으로 고정
                    )
                    continue

        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)
        prev_ticks_right[i] = tick


# =========================
# 포트 오픈 (왼팔 / 오른팔 각각)
# =========================
portHandler_left = PortHandler(DEVICENAME_LEFT)
packetHandler_left = PacketHandler(PROTOCOL_END)

portHandler_right = PortHandler(DEVICENAME_RIGHT)
packetHandler_right = PacketHandler(PROTOCOL_END)

if not portHandler_left.openPort():
    print("왼팔 포트 열기 실패")
    quit()
if not portHandler_left.setBaudRate(BAUDRATE):
    print("왼팔 보레이트 실패")
    quit()

if not portHandler_right.openPort():
    print("오른팔 포트 열기 실패")
    quit()
if not portHandler_right.setBaudRate(BAUDRATE):
    print("오른팔 보레이트 실패")
    quit()

# =========================
# 모터 초기화
# =========================
for m in MOTORS_LEFT:
    packetHandler_left.write1ByteTxRx(
        portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    packetHandler_left.write1ByteTxRx(portHandler_left, m, ADDR_ACCELERATION, 50)

for m in MOTORS_RIGHT:
    packetHandler_right.write1ByteTxRx(
        portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    packetHandler_right.write1ByteTxRx(portHandler_right, m, ADDR_ACCELERATION, 50)

packetHandler_right.write1ByteTxRx(
    portHandler_right, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
packetHandler_right.write1ByteTxRx(
    portHandler_right, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
packetHandler_right.write1ByteTxRx(portHandler_right, PAN_ID, ADDR_ACCELERATION, 50)
packetHandler_right.write1ByteTxRx(portHandler_right, TILT_ID, ADDR_ACCELERATION, 100)

# =========================
# 팬틸트 중앙 이동
# =========================
scs_write_pos(packetHandler_right, portHandler_right, PAN_ID, 511)
time.sleep(0.5)
scs_write_pos(packetHandler_right, portHandler_right, TILT_ID, 511)
time.sleep(0.5)

# =========================
# ADC thread (양팔 공용, 1개만 실행)
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

# [추가] CRC-16 / 시퀀스 검증 로직 자체 테스트 (실전 데이터 아님, 구현 검증용)
crc16_self_test()
sequence_validator_self_test()

# ===========================
adc_filter.reset()
print("시작")

# =========================
# 메인 루프
# =========================
try:
    while True:
        # ✅ 수정 1: 타임아웃 감지 로직을 루프 최상단으로 이동 (continue에 무시되지 않도록)
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

        # =====================================================
        # 시작 안정화 대기 (양팔 모두 유효 데이터 들어올 때까지)
        # =====================================================
        if not (system_ready_left and system_ready_right):
            if not system_ready_left:
                startup_count_left += 1
                if startup_count_left % 50 == 0:  # 1초마다 한 번씩만 출력 (스팸 방지)
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
                        # [추가] prev_ticks도 같은 시점 값으로 초기화 → 첫 프레임 훅 이동 방지
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
                # [수정] system_ready_left가 실제로 True가 된 순간에만 출력
                # (기존엔 이 print가 조건 없이 매 루프 찍혀서, 대기 중에도
                #  "왼팔 준비 완료"가 스팸처럼 반복 출력되는 버그가 있었음)
                if system_ready_left:
                    print(">>> 왼팔 준비 완료")

            # ✅ 수정 2: 오른팔도 데이터가 0보다 큰 정상 값인지 검사하도록 수정
            if not system_ready_right:
                startup_count_right += 1
                if startup_count_right >= STARTUP_WAIT_RIGHT and any(
                    0 < parsed[i + 7] < FLOATING_THRESHOLD for i in range(7)
                ):
                    system_ready_right = True
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
                                + ratio
                                * (GRIPPER_POS_OPEN_RIGHT - GRIPPER_POS_CLOSE_RIGHT)
                            )
                        else:
                            init_tick = int(parsed[i + 7])
                            if i in REVERSE_CHANNELS_RIGHT:
                                init_tick = 4095 - init_tick
                            prev_ticks_right[i] = init_tick
                    print(">>> 오른팔 준비 완료")

            time.sleep(0.02)
            continue

        # =====================================================
        # [추가] 통신 타임아웃 감지 → 양팔 강제 ERROR
        # =====================================================
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

        # =========================
        # 왼팔 / 오른팔 각각 독립 처리
        # =========================
        process_left_arm(portHandler_left, packetHandler_left)
        process_right_arm(portHandler_right, packetHandler_right)

        lift_state = update_lift_state(parsed[16])  # ← 추가
        # =========================
        # 팬틸트 업데이트
        # =========================
        update_pantilt(
            parsed, sw_toggle, packetHandler_right, portHandler_right, PAN_ID, TILT_ID
        )
        update_wheels(parsed, sw1_toggle)  # 7월 30일

        # =========================
        # 출력
        # =========================
        print("\033[F", end="")

        left_raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        right_raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(
            f"----------------------------------------------\n"
            f"[팔]  L:{left_raw_str}\n"
            f"      R:{right_raw_str}\n"
            f"[상태] L_STATE:{current_state_left:5s}(prev:{prev_state_left:5s})  "
            f"R_STATE:{current_state_right:5s}(prev:{prev_state_right:5s})\n"
            f"[팬틸트] SW:{sw_toggle}  PAN:{pantilt_safe2.pan_pos:4d}  TILT:{pantilt_safe2.tilt_pos:4d}\n"
            f"[상하이동] LIFT:{lift_state:+d} (raw:{parsed[16]:4d})\n"
            f"[바퀴조이스틱] SW1:{sw1_toggle}  ind2:{parsed[17]:4d}  ind3:{parsed[18]:4d}\n"
            f"[키캡] [{int(key1_pressed)}{int(key2_pressed)}{int(key3_pressed)}{int(key4_pressed)}{int(key5_pressed)}]\n"
            f"----------------------------------------------"
        )

        # =====================================================
        # [추가] 라즈베리파이로 UDP 전송 (최종 서보 tick 값 기준)
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
# 종료
# =========================
running = False

# =========================
# [추가] 종료 전 양팔을 안전 자세로 이동
# =========================
NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]  # 모터 1~7
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]  # 모터 9~15

for i, m in enumerate(MOTORS_LEFT):
    target = NEUTRAL_TICKS_LEFT[i]
    packetHandler_left.write2ByteTxRx(portHandler_left, m, ADDR_GOAL_POSITION, target)

for i, m in enumerate(MOTORS_RIGHT):
    target = NEUTRAL_TICKS_RIGHT[i]
    packetHandler_right.write2ByteTxRx(portHandler_right, m, ADDR_GOAL_POSITION, target)

time.sleep(1.5)  # 양팔이 실제로 목표 위치까지 이동할 시간 확보

for m in MOTORS_LEFT:
    packetHandler_left.write1ByteTxRx(
        portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )

for m in MOTORS_RIGHT:
    packetHandler_right.write1ByteTxRx(
        portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )

packetHandler_right.write1ByteTxRx(
    portHandler_right, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)
packetHandler_right.write1ByteTxRx(
    portHandler_right, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)

portHandler_left.closePort()
portHandler_right.closePort()
adc_filter.report()

print("종료")
