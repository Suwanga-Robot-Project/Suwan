import serial
import time
import struct
import statistics

# =====================================================
# [진단 전용] 정지 시 명령값 흔들림 + 루프 주기 동시 확인
# Nexus_5.py의 ADC 파싱 + process_left_arm 로직을 그대로 가져와서,
# "팔을 가만히 두고 있을 때 tick 값이 실제로 바뀌는지"와
# "루프가 얼마나 규칙적으로 도는지"를 한 번에 로그로 남깁니다.
#
# 사용법: 팔을 가만히 든 상태(조종기도 안 만짐)로 이 스크립트를
# 30초~1분 정도 돌려보세요.
# =====================================================

PORT_ADC = "COM13"
BAUD_ADC = 115200

PACKET_HEADER = b"\xaa\x55"
PACKET_SIZE = 51
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBH")


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


# =========================
# 왼팔 tick 계산 설정 (Nexus_5.py와 동일)
# =========================
REVERSE_CHANNELS_LEFT = [5]
EMA_ALPHA_ARM_LEFT = [0.35, 0.35, 0.35, 0.3, 0.4, 0.7]  # ★ 여기 [5]=0.7이 의심 지점
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 28
DEAD_ZONE_EXIT_LEFT = 40
MAX_DELTA_LEFT = 70
MAX_ACCEL_LEFT = 15

GRIPPER_ADC_MIN_LEFT = 145
GRIPPER_ADC_MAX_LEFT = 1270
GRIPPER_POS_OPEN_LEFT = 4100
GRIPPER_POS_CLOSE_LEFT = 500

FLOATING_THRESHOLD = 4080

ema_values_left = [None] * 7
prev_ticks_left = [None] * 7
prev_delta_left = [0] * 7
in_dead_zone_left = [False] * 7
dead_zone_anchor_left = [None] * 7

parsed = [0] * 17
adc_raw = [0] * 23


def parse_packet(raw_packet):
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
        return False

    mux_vals = [
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
        adc_raw[i] = mux_vals[i]
    adc_raw[16] = ind0

    for i in range(7):
        parsed[i] = adc_raw[i + 1]
    return True


def process_left_arm_tick(i):
    """채널 i(0~6)의 tick 값을 계산. 실제 process_left_arm과 동일 로직."""
    raw = parsed[i]
    if raw >= FLOATING_THRESHOLD:
        return None

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

        diff = abs(tick - prev_ticks_left[i])
        if in_dead_zone_left[i]:
            diff_from_anchor = abs(tick - dead_zone_anchor_left[i])
            if diff_from_anchor <= DEAD_ZONE_EXIT_LEFT:
                return prev_ticks_left[i]  # 데드존 안, tick 갱신 안 함
            else:
                in_dead_zone_left[i] = False
        else:
            if diff <= DEAD_ZONE_ENTER_LEFT:
                in_dead_zone_left[i] = True
                dead_zone_anchor_left[i] = tick
                return prev_ticks_left[i]

    prev_ticks_left[i] = tick
    return tick


try:
    ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    print(f">>> {PORT_ADC} 오픈 성공, 정지 상태로 팔을 가만히 두고 관찰하세요.\n")
except Exception as e:
    print("시리얼 열기 실패:", e)
    raise SystemExit(1)

buf = bytearray()
loop_intervals = []
last_loop_time = time.time()
tick_change_count = [0] * 7  # 채널별 tick이 바뀐 횟수
total_frames = 0

CH_NAMES = ["1", "2", "3", "4", "5", "6(그리퍼)"]

try:
    while True:
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
            ok = parse_packet(raw_packet)
            del buf[:PACKET_SIZE]

            if not ok:
                continue

            now = time.time()
            interval = now - last_loop_time
            loop_intervals.append(interval)
            last_loop_time = now
            total_frames += 1

            prev_snapshot = list(prev_ticks_left)
            for i in range(7):
                process_left_arm_tick(i)
                if (
                    prev_snapshot[i] is not None
                    and prev_ticks_left[i] != prev_snapshot[i]
                ):
                    tick_change_count[i] += 1

            if total_frames % 25 == 0:
                tick_str = " ".join(
                    f"{t if t is not None else '-':>5}" for t in prev_ticks_left
                )
                print(f"\rframe={total_frames:5d} ticks=[{tick_str}]", end="")

except KeyboardInterrupt:
    pass
finally:
    ser.close()

print("\n\n>>> 진단 종료 — 결과 요약")
print("-" * 60)

if len(loop_intervals) > 1:
    ms = [x * 1000 for x in loop_intervals]
    print(
        f"[루프 주기] 평균 {statistics.mean(ms):.2f}ms  표준편차 {statistics.stdev(ms):.2f}ms  최대 {max(ms):.2f}ms"
    )

print(f"\n[채널별 tick 변경 횟수] (총 {total_frames}프레임 중)")
for i in range(7):
    pct = tick_change_count[i] / total_frames * 100 if total_frames else 0
    flag = "  ⚠ 정지 중인데 계속 변함" if pct > 5 else ""
    print(f"  채널{CH_NAMES[i]}: {tick_change_count[i]:5d}회 ({pct:5.1f}%){flag}")

print("-" * 60)
print("해석: 팔을 안 만졌는데도 특정 채널의 변경 비율이 높다면(특히 5%가 넘으면),")
print("그 채널은 EMA/데드존 튜닝이 필요한 소프트웨어 원인일 가능성이 높습니다.")
print("모든 채널이 0%에 가까운데도 서보가 떨었다면, 그건 서보 자체 특성입니다.")
