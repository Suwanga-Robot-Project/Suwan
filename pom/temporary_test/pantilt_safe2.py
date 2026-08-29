from scservo_sdk import *

STEP = 3

PAN_MIN = 0
PAN_MAX = 1023

TILT_MIN = 200
TILT_MAX = 700

pan_pos = 511
tilt_pos = 511

# =====================================================
# 조이스틱 EMA 필터
# α = 0.2
# =====================================================
EMA_ALPHA_JOY = 0.2
ema_ind0 = None
ema_ind1 = None

# =====================================================
# [추가] 대각선 간섭(crosstalk) 방지 — 우세 축 판정
# 한쪽 편차가 다른 쪽보다 이 배율 이상 크면, 그 축만 반응시키고
# 반대쪽은 무시함. 진짜 대각선 의도(편차가 서로 비슷할 때)는
# 원래대로 두 축 다 반응.
# =====================================================
AXIS_DOMINANCE_RATIO = 1.8

# =====================================================
# [추가] 틸트 반응 속도 보정
# 틸트 축이 물리적으로 부하가 커서(머리/카메라 무게 등) 팬보다
# 느리게 따라오는 걸 보완하기 위해, 틸트만 step 계산에 배속을 곱함.
# =====================================================
TILT_SPEED_GAIN = 3.0


def scs_write_pos(scs_packet, portHandler, servo_id, pos):

    pos = SCS_TOSCS(pos, 0)

    pos_L = SCS_LOBYTE(pos)
    pos_H = SCS_HIBYTE(pos)

    scs_packet.writeTxRx(portHandler, servo_id, 42, 2, [pos_H, pos_L])


def update_pantilt(adc, sw, scs_packet, portHandler, PAN_ID, TILT_ID):
    """
    [원본 시그니처 복원] 로컬 서보(COM14)에 직접 쓰는 버전.
    scs_packet/portHandler를 받아 계산된 pan_pos/tilt_pos를 그대로
    서보에 씀 (원본 Nexus_5.py의 유선 직결 구조에 맞춤).

    [추가] 대각선 crosstalk 방지 + 틸트 속도 보정 적용됨.
    """

    global pan_pos
    global tilt_pos
    global ema_ind0
    global ema_ind1

    # print(f"DEBUG - adc[14]:{adc[14]} adc[15]:{adc[15]} sw:{sw}")

    # =========================
    # SW OFF
    # =========================
    if sw != 1:
        return

    # =========================
    # IND INPUT
    # =========================
    raw_ind0 = adc[14]
    raw_ind1 = adc[15]

    # =====================================================
    # EMA 필터 적용
    # =====================================================
    if ema_ind0 is None:
        ema_ind0 = float(raw_ind0)
        ema_ind1 = float(raw_ind1)
    else:
        ema_ind0 = EMA_ALPHA_JOY * raw_ind0 + (1 - EMA_ALPHA_JOY) * ema_ind0
        ema_ind1 = EMA_ALPHA_JOY * raw_ind1 + (1 - EMA_ALPHA_JOY) * ema_ind1

    ind0 = int(ema_ind0)
    ind1 = int(ema_ind1)
    print(f"IND0:{ind0} IND1:{ind1}")

    # =========================
    # CENTER 기준값
    # =========================
    center = 2000
    deadzone = 150

    # =====================================================
    # STEP 가변화 함수 (틸트는 gain 배속 적용 가능)
    # deadzone 바깥으로 벗어난 거리에 비례해서 속도 결정
    # 최소 1, 최대 8 (틸트는 gain 적용 시 그 이상까지 허용)
    # =====================================================
    def calc_step(val, gain=1.0):
        deviation = abs(val - center) - deadzone
        if deviation <= 0:
            return 0
        step = int(deviation / 1900 * 8 * gain)
        return max(1, min(int(8 * gain), step))

    # =====================================================
    # [추가] 우세 축 판정 — 대각선 간섭 방지
    # =====================================================
    dev_pan = abs(ind0 - center)
    dev_tilt = abs(ind1 - center)

    pan_active = dev_pan > deadzone
    tilt_active = dev_tilt > deadzone

    if pan_active and tilt_active:
        if dev_pan > dev_tilt * AXIS_DOMINANCE_RATIO:
            tilt_active = False
        elif dev_tilt > dev_pan * AXIS_DOMINANCE_RATIO:
            pan_active = False
        # else: 편차가 서로 비슷 → 진짜 대각선 의도로 보고 둘 다 허용

    # =========================
    # PAN (IND0)
    # =========================
    if pan_active:
        if ind0 > center + deadzone:
            pan_pos -= calc_step(ind0)
        elif ind0 < center - deadzone:
            pan_pos += calc_step(ind0)

    # =========================
    # TILT (IND1) — 속도 보정(gain) 적용
    # =========================
    if tilt_active:
        if ind1 > center + deadzone:
            tilt_pos -= calc_step(ind1, gain=TILT_SPEED_GAIN)
        elif ind1 < center - deadzone:
            tilt_pos += calc_step(ind1, gain=TILT_SPEED_GAIN)

    # =========================
    # LIMIT
    # =========================
    if pan_pos > PAN_MAX:
        pan_pos = PAN_MAX
    elif pan_pos < PAN_MIN:
        pan_pos = PAN_MIN

    if tilt_pos > TILT_MAX:
        tilt_pos = TILT_MAX
    elif tilt_pos < TILT_MIN:
        tilt_pos = TILT_MIN

    # =========================
    # WRITE — 로컬 서보(COM14)에 직접 반영
    # =========================
    scs_write_pos(scs_packet, portHandler, PAN_ID, pan_pos)
    scs_write_pos(scs_packet, portHandler, TILT_ID, tilt_pos)
