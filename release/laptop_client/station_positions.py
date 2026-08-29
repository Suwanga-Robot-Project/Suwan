"""
teach_position_by_feedback.py로 실측한 스테이션별 tick 값.
순서: [모터1, 모터2, 모터3, 모터4, 모터5, 모터6, 모터7(그리퍼)]

키캡 매핑 확정: 1=오른팔 미세, 2=오른팔 니퍼, 3=왼팔 기본(디폴트), 4=왼팔 바이스

⚠️ 그리퍼 4종(미세/니퍼/기본/바이스) 모두 MAX_OPEN/MAX_CLOSE 값이 서로 다름 (실측완료)
    → LEFT/RIGHT 대신 스테이션 이름으로 변수를 구분함 (혼동 방지)
"""

# ===== NEUTRAL 자세 — 팔을 일자로 쭉 편 자세, 리프트 상하이동 중 유지 =====
# (실측완료)
NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]

# ===== 왼팔 스테이션 (실측완료) =====
DEFAULT_TICKS_LEFT = [1421, 1034, 1276, 1593, 1864, 2129, 2563]
VISE_TICKS_LEFT = [1460, 1153, 1275, 1580, 1790, 2183, 2156]

# ===== 오른팔 스테이션 (실측완료) =====
FINE_TICKS_RIGHT = [2540, 1017, 3078, 1586, 2075, 2183, 2549]
NIPPER_TICKS_RIGHT = [2468, 858, 3078, 1736, 1939, 2224, 1718]

# =====================================================================
# ===== 그리퍼별 최대 개방/조임 값 (탈거·부착 시 사용, 실측완료) =====
# =====================================================================
# 탈거 원리: 스테이션에 놓인 채로 그리퍼를 "최대로 조인 뒤" → "최대로 펼 때까지"
# 부착 원리: 그리퍼를 "연 상태로" 목표 위치까지 접근한 뒤 → "최대로 인다"
#
# ⚠️ 스테이션(그리퍼 종류)마다 값이 전부 다르므로, 팔이 아니라 스테이션
#    이름을 기준으로 이름 붙임 (LEFT/RIGHT 혼동 방지)

# ===== 1번: 오른팔 미세그리퍼 =====
FINE_MAX_OPEN_RIGHT = 2549  # 미세 그리퍼: 최대로 펼 때
FINE_MAX_CLOSE_RIGHT = 3354  # 미세 그리퍼: 최대로 조일 때

# ===== 2번: 오른팔 니퍼그리퍼 =====
NIPPER_MAX_OPEN_RIGHT = 1898  # 니퍼: 최대로 펼 때
NIPPER_MAX_CLOSE_RIGHT = 3393  # 니퍼: 최대로 조일 때

# ===== 3번: 왼팔 기본그리퍼 =====
DEFAULT_MAX_OPEN_LEFT = 2658  # 기본 그리퍼: 최대로 펼 때
DEFAULT_MAX_CLOSE_LEFT = 3154  # 기본 그리퍼: 최대로 조일 때

# ===== 4번: 왼팔 바이스그리퍼 =====
VISE_MAX_OPEN_LEFT = 2156  # 바이스 그리퍼: 최대로 펼 때
VISE_MAX_CLOSE_LEFT = 3254  # 바이스 그리퍼: 최대로 조일 때

# ===== 니퍼 안전범위 (아직 실측 전) =====
# 니퍼는 다른 그리퍼보다 이동거리 한계가 짧아서 보호 필요
# 예: (350, 800) — 니퍼는 절대 이 범위 밖에서 움직이면 안 됨
NIPPER_SAFE_TICK_RANGE = (1718, 3593)  # (MIN, MAX)

# =====================================================================
# ===== 반대편 팔 안전 대기자세 (충돌 방지용, 새로 추가) =====
# =====================================================================
# 한쪽 팔이 그리퍼교체(하강 포함) 하는 동안, 반대편 팔은 라이브 조종 위치에
# 그대로 있으면 스테이션 프레임과 부딪힐 수 있음. 이를 막기 위해 반대편 팔을
# "NEUTRAL 자세에서 1번 모터만 다른 값으로 바꾼" 안전 위치로 잠깐 옮겨두고,
# 스왑이 끝나면 원래 있던 자리로 복귀시킴.
#
# 아직 미실측 — 측정 방법:
#   1) 팔을 NEUTRAL 자세로 두기
#   2) 1번 모터만 손으로 돌려서, 반대편(활성 팔)이 어느 스테이션으로 가든
#      절대 충돌 안 나는 각도를 찾기 (4개 스테이션 전부 시험해보고 확인 권장)
#   3) 그 각도의 1번 모터 present position을 여기에 기록

SAFE_RETREAT_MOTOR1_LEFT = 1749  # 왼팔 안전자세 1번 모터 tick (실측 필요)
SAFE_RETREAT_MOTOR1_RIGHT = 2753  # 오른팔 안전자세 1번 모터 tick (실측 필요)


def get_safe_retreat_ticks(arm_side):
    """
    NEUTRAL에서 1번 모터만 SAFE_RETREAT_MOTOR1_* 값으로 바꾼 안전 자세 tick 반환.
    아직 미실측이면 None 반환 (호출부에서 건너뛰고 경고 출력하도록 처리).
    """
    neutral = NEUTRAL_TICKS_LEFT if arm_side == "left" else NEUTRAL_TICKS_RIGHT
    motor1_override = (
        SAFE_RETREAT_MOTOR1_LEFT if arm_side == "left" else SAFE_RETREAT_MOTOR1_RIGHT
    )
    if motor1_override is None:
        return None
    ticks = list(neutral)
    ticks[0] = motor1_override
    return ticks


# =====================================================================
# ===== B안: 직접 전환(클리어런스) — 1↔2(오른팔), 3↔4(왼팔)만 해당 =====
# =====================================================================
# NEUTRAL을 거치지 않고, 그리퍼를 놓은(탈거한) 직후 살짝 든 상태로 바로
# 옆 스테이션 방향으로 팔을 이동하는 "중간 지점" 순차 시퀀스.
#
# ⚠️ 형식이 [tick1~tick7] 전체 스냅샷이 아니라, "어떤 모터를 몇 tick으로,
#    얼마나 대기한 뒤에 움직일지"를 순서대로 나열한 리스트입니다.
#
# 형식: [(모터_인덱스, 목표tick, 이번_스텝_실행후_대기초), ...]
#   모터_인덱스는 0부터 시작 (1번 모터=0, 2번 모터=1, ..., 7번 그리퍼=6)
#
# ⚠️ 팔마다 순서가 다름:
#   오른팔(1↔2, fine/nipper): 1번 모터 먼저 → 4번 모터 나중
#   왼팔(3↔4, default/vise): 4번 모터 먼저 → 1번 모터 나중

DIRECT_SWAP_PAIRS_RIGHT = {("fine", "nipper"), ("nipper", "fine")}
DIRECT_SWAP_PAIRS_LEFT = {("default", "vise"), ("vise", "default")}

DIRECT_SWAP_CLEARANCE_RIGHT = {
    # 오른팔: 1번 모터 먼저 → 4번 모터 나중
    ("fine", "nipper"): [
        (0, 2142, 1.0),  # 1번 모터(인덱스0) 먼저 회전, 1초 대기
        (3, 2427, 0.3),  # 이어서 4번 모터(인덱스3) 살짝 들기, 0.3초 대기
    ],
    ("nipper", "fine"): [
        (0, 2007, 1.0),
        (3, 2698, 0.3),
    ],
}
DIRECT_SWAP_CLEARANCE_LEFT = {
    # 왼팔: 4번 모터 먼저 → 1번 모터 나중
    ("default", "vise"): [
        (3, 2414, 1.0),  # 4번 모터(인덱스3) 먼저 살짝 들기, 1초 대기
        (0, 2156, 0.3),  # 이어서 1번 모터(인덱스0) 회전, 0.3초 대기
    ],
    ("vise", "default"): [
        (3, 2061, 1.0),
        (0, 2129, 0.3),
    ],
}


# ===== Helper 함수들 =====


def is_direct_swap_pair(arm_side, held, target):
    """held/target 둘 다 있고, 1↔2 또는 3↔4 짝에 해당하면 True."""
    if held is None or target is None:
        return False
    pairs = DIRECT_SWAP_PAIRS_RIGHT if arm_side == "right" else DIRECT_SWAP_PAIRS_LEFT
    return (held, target) in pairs


def get_direct_swap_clearance(arm_side, held, target):
    """직접 전환(클리어런스) 순차이동 시퀀스 가져오기.
    반환: [(모터_인덱스, 목표tick, 대기초), ...] 형태의 리스트 (또는 None)"""
    table = (
        DIRECT_SWAP_CLEARANCE_RIGHT
        if arm_side == "right"
        else DIRECT_SWAP_CLEARANCE_LEFT
    )
    return table.get((held, target))


def get_station_ticks(arm_side, station_name):
    """팔과 그리퍼 이름으로 스테이션 tick 조회. (원본, 보정 안 됨)"""
    table = {
        ("left", "default"): DEFAULT_TICKS_LEFT,
        ("left", "vise"): VISE_TICKS_LEFT,
        ("right", "fine"): FINE_TICKS_RIGHT,
        ("right", "nipper"): NIPPER_TICKS_RIGHT,
    }
    return table.get((arm_side, station_name))


# =====================================================================
# ===== 백래시 보정값 (2026-08-09 position_accuracy_diagnostic.py 실측) =====
# =====================================================================
# 목표 tick으로 명령해도 실제로는 항상 비슷한 크기/방향으로 어긋나는 현상이
# 4~5회 반복 측정에서 재현성 있게 관측됨 (기어 백래시로 추정).
# 오차 = 실제값 - 명령값 (관측된 값). 아래처럼 보정해서 보냄:
#   보정 명령값 = 원래 목표값 - 오차
# → 오차만큼 반대 방향으로 미리 밀어서 보내면, 실제 도달값이 원하는 목표에 더 가까워짐.
#
# 형식: (arm_side, station_name): [모터별 오차, ...] — get_station_ticks()가
# 반환하는 tick 리스트와 순서가 동일함(7개).
BACKLASH_OFFSET = {
    ("right", "fine"): [17, 0, -5, -13, 4, -2, 4],
    ("right", "nipper"): [21, 3, 3, -15, 4, -2, -2],
    ("left", "default"): [-16, 7, -4, -9, -2, -3, -3],
    ("left", "vise"): [-18, -7, -4, -10, 3, -6, -3],
}


def get_corrected_station_ticks(arm_side, station_name):
    """
    get_station_ticks()의 값에 실측된 백래시 오차를 보정해서 반환.
    실제 로봇 이동에는 이 함수를 쓰는 게 좋음(get_station_ticks는 원본 참고용).
    보정값이 없는 조합(아직 진단 안 한 스테이션)이면 원본 그대로 반환.
    """
    raw = get_station_ticks(arm_side, station_name)
    if raw is None:
        return None
    offset = BACKLASH_OFFSET.get((arm_side, station_name))
    if offset is None:
        return raw
    return [
        int(t - o) if (t is not None and o is not None) else t
        for t, o in zip(raw, offset)
    ]


def get_gripper_max_open(arm_side, gripper_name):
    """그리퍼 종류별 최대 개방 tick 조회."""
    table = {
        ("left", "default"): DEFAULT_MAX_OPEN_LEFT,
        ("left", "vise"): VISE_MAX_OPEN_LEFT,
        ("right", "fine"): FINE_MAX_OPEN_RIGHT,
        ("right", "nipper"): NIPPER_MAX_OPEN_RIGHT,
    }
    return table.get((arm_side, gripper_name))


def get_gripper_max_close(arm_side, gripper_name):
    """그리퍼 종류별 최대 조임 tick 조회."""
    table = {
        ("left", "default"): DEFAULT_MAX_CLOSE_LEFT,
        ("left", "vise"): VISE_MAX_CLOSE_LEFT,
        ("right", "fine"): FINE_MAX_CLOSE_RIGHT,
        ("right", "nipper"): NIPPER_MAX_CLOSE_RIGHT,
    }
    return table.get((arm_side, gripper_name))


def clamp_for_nipper(gripper_tick):
    """니퍼 장착 시에만 그리퍼(7번 모터) tick을 안전범위로 제한."""
    lo, hi = NIPPER_SAFE_TICK_RANGE
    if lo is None or hi is None:
        raise ValueError("NIPPER_SAFE_TICK_RANGE가 아직 실측되지 않았습니다")
    return max(lo, min(hi, gripper_tick))
