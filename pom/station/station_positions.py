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
DEFAULT_TICKS_LEFT = [
    1437,
    1017,
    1272,
    1584,
    1862,
    2115,
    2644,
]
VISE_TICKS_LEFT = [
    1424,
    1139,
    1275,
    1580,
    1790,
    2102,
    2631,
]

# ===== 오른팔 스테이션 (실측완료) =====
FINE_TICKS_RIGHT = [
    2522,
    990,
    3078,
    1586,
    2020,
    2142,
    2644,
]
NIPPER_TICKS_RIGHT = [
    2549,
    845,
    3119,
    1573,
    1925,
    2183,
    1790,
]

# =====================================================================
# ===== 그리퍼별 최대 개방/조임 값 (탈거·부착 시 사용, 실측완료) =====
# =====================================================================
# 탈거 원리: 스테이션에 놓인 채로 그리퍼를 "최대로 조인 뒤" → "최대로 펼 때까지"
# 부착 원리: 그리퍼를 "연 상태로" 목표 위치까지 접근한 뒤 → "최대로 조인다"
#
# ⚠️ 스테이션(그리퍼 종류)마다 값이 전부 다르므로, 팔이 아니라 스테이션
#    이름을 기준으로 이름 붙임 (LEFT/RIGHT 혼동 방지)

# ===== 1번: 오른팔 미세그리퍼 =====
FINE_MAX_OPEN_RIGHT = 2292  # 미세 그리퍼: 최대로 펼 때
FINE_MAX_CLOSE_RIGHT = 3742  # 미세 그리퍼: 최대로 조일 때

# ===== 2번: 오른팔 니퍼그리퍼 =====
NIPPER_MAX_OPEN_RIGHT = 2048  # 니퍼: 최대로 펼 때
NIPPER_MAX_CLOSE_RIGHT = 3268  # 니퍼: 최대로 조일 때

# ===== 3번: 왼팔 기본그리퍼 =====
DEFAULT_MAX_OPEN_LEFT = 1749  # 기본 그리퍼: 최대로 펼 때
DEFAULT_MAX_CLOSE_LEFT = 3254  # 기본 그리퍼: 최대로 조일 때

# ===== 4번: 왼팔 바이스그리퍼 =====
VISE_MAX_OPEN_LEFT = 1478  # 바이스 그리퍼: 최대로 펼 때
VISE_MAX_CLOSE_LEFT = 4095  # 바이스 그리퍼: 최대로 조일 때

# ===== 니퍼 안전범위 (아직 실측 전) =====
# 니퍼는 다른 그리퍼보다 이동거리 한계가 짧아서 보호 필요
# 예: (350, 800) — 니퍼는 절대 이 범위 밖에서 움직이면 안 됨
NIPPER_SAFE_TICK_RANGE = (None, None)  # (MIN, MAX)

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
#
# 짝이 아닌 조합(예: 처음 빈손으로 뭔가 부착할 때)은 이 경로를 안 쓰고
# 기존 NEUTRAL 경유 방식을 그대로 씀.
#
# 아직 미실측 — 측정 방법 (오른팔 예시):
#   1) 탈거 직후 상태(그리퍼 MAX_OPEN, 나머지는 스테이션 tick)에서 시작
#   2) 1번 모터만 손으로 조금 돌려서 present position 읽기 → 그 값을
#      (0, 읽은값, 원하는 대기초) 로 기록
#   3) 이어서 4번 모터를 살짝 들어올려서 present position 읽기
#      → (3, 읽은값, 원하는 대기초) 로 기록
#   (왼팔은 순서만 반대: 4번 먼저 → 1번 나중)

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
    """팔과 그리퍼 이름으로 스테이션 tick 조회."""
    table = {
        ("left", "default"): DEFAULT_TICKS_LEFT,
        ("left", "vise"): VISE_TICKS_LEFT,
        ("right", "fine"): FINE_TICKS_RIGHT,
        ("right", "nipper"): NIPPER_TICKS_RIGHT,
    }
    return table.get((arm_side, station_name))


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
