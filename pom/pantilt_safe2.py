from scservo_sdk import *

STEP = 3

PAN_MIN = 0
PAN_MAX = 1023

TILT_MIN = 0
TILT_MAX = 1023

pan_pos = 511
tilt_pos = 511

# =====================================================
# [추가] 조이스틱 EMA 필터
# α = 0.2 (조이스틱은 부드럽게)
# =====================================================
EMA_ALPHA_JOY = 0.2
ema_ind0 = None
ema_ind1 = None


def scs_write_pos(scs_packet, portHandler, servo_id, pos):

    pos = SCS_TOSCS(pos, 0)

    pos_L = SCS_LOBYTE(pos)
    pos_H = SCS_HIBYTE(pos)

    scs_packet.writeTxRx(portHandler, servo_id, 42, 2, [pos_H, pos_L])


def update_pantilt(adc, sw, scs_packet, portHandler, PAN_ID, TILT_ID):

    global pan_pos
    global tilt_pos
    global ema_ind0
    global ema_ind1

    print(f"DEBUG - adc[14]:{adc[14]} adc[15]:{adc[15]} sw:{sw}")  # 여기 추가

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
    # [추가] EMA 필터 적용
    # =====================================================
    if ema_ind0 is None:
        ema_ind0 = float(raw_ind0)
        ema_ind1 = float(raw_ind1)
    else:
        ema_ind0 = EMA_ALPHA_JOY * raw_ind0 + (1 - EMA_ALPHA_JOY) * ema_ind0
        ema_ind1 = EMA_ALPHA_JOY * raw_ind1 + (1 - EMA_ALPHA_JOY) * ema_ind1

    ind0 = int(ema_ind0)
    ind1 = int(ema_ind1)
    # 추가------------------
    print(f"IND0:{ind0} IND1:{ind1}")

    # =========================
    # CENTER 기준값
    # =========================
    center = 2000
    deadzone = 150

    # =====================================================
    # [추가] STEP 가변화 함수
    # deadzone 바깥으로 벗어난 거리에 비례해서 속도 결정
    # 최소 1, 최대 8 (너무 빠르면 제어 어려워서 제한)
    # =====================================================
    def calc_step(val):
        deviation = abs(val - center) - deadzone  # deadzone 제거한 순수 이탈량
        if deviation <= 0:
            return 0
        step = int(deviation / 1900 * 8)  # 최대 이탈(1900) 기준 최대 8
        return max(1, min(8, step))  # 1~8 클램핑

    # =========================
    # PAN (IND0)
    # =========================
    if ind0 > center + deadzone:
        pan_pos -= calc_step(ind0)
    elif ind0 < center - deadzone:
        pan_pos += calc_step(ind0)

    # =========================
    # TILT (IND1)
    # =========================
    if ind1 > center + deadzone:
        tilt_pos -= calc_step(ind1)
    elif ind1 < center - deadzone:
        tilt_pos += calc_step(ind1)

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
    # WRITE
    # =========================
    scs_write_pos(scs_packet, portHandler, PAN_ID, pan_pos)
    scs_write_pos(scs_packet, portHandler, TILT_ID, tilt_pos)
