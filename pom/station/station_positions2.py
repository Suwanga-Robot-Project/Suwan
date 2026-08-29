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
DEFAULT_TICKS_LEFT = [1406, 1400, 1855, 1790, 245, 1722, 0]
VISE_TICKS_LEFT = [1234, 1348, 1850, 1735, 258, 1774, 0]

# ===== 오른팔 스테이션 (실측완료) =====
FINE_TICKS_RIGHT = [2396, 1195, 2707, 1607, 3446, 1757, 0]
NIPPER_TICKS_RIGHT = [2536, 1204, 2686, 1763, 3450, 1654, 0]

# =====================================================================
# ===== 그리퍼별 최대 개방/조임 값 (탈거·부착 시 사용, 실측완료) =====
# =====================================================================
# 탈거 원리: 스테이션에 놓인 채로 그리퍼를 "최대로 조인 뒤" → "최대로 펼 때까지"
# 부착 원리: 그리퍼를 "연 상태로" 목표 위치까지 접근한 뒤 → "최대로 인다"
#
# ⚠️ 스테이션(그리퍼 종류)마다 값이 전부 다르므로, 팔이 아니라 스테이션
#    이름을 기준으로 이름 붙임 (LEFT/RIGHT 혼동 방지)

# ===== 1번: 오른팔 미세그리퍼 =====
FINE_MAX_OPEN_RIGHT = 906  # 미세: 최대 개방 (gripper_dir_check 실측)
FINE_MAX_CLOSE_RIGHT = 3506  # 미세: 최대 조임 (허공 기준 — 아래 주의 참고)

# ===== 2번: 오른팔 니퍼그리퍼 =====
NIPPER_MAX_OPEN_RIGHT = 1208  # 니퍼: 최대 개방 (실측)
NIPPER_MAX_CLOSE_RIGHT = 3083  # 니퍼: 최대 조임 (실측)

# ===== 3번: 왼팔 기본그리퍼 =====
DEFAULT_MAX_OPEN_LEFT = 1172  # 기본: 최대 개방 (실측)
DEFAULT_MAX_CLOSE_LEFT = 3769  # 기본: 최대 조임 (실측)

# ===== 4번: 왼팔 바이스그리퍼 =====
VISE_MAX_OPEN_LEFT = 1404  # 바이스: 최대 개방 (실측)
VISE_MAX_CLOSE_LEFT = 4001  # 바이스: 최대 조임 (실측)

# ===== (폐기) 구 니퍼 안전범위 =====
# 값이 니퍼 실제 가동범위(약 2048~3268)보다 넓어 아무것도 막지 못했음.
# 그리퍼 안전범위는 아래 GRIPPER_SAFE_RANGE 로 이관됨.

# =====================================================================
# ===== 반대편 팔 안전 대기자세 (충돌 방지용) =====
# =====================================================================
# 한쪽 팔이 그리퍼교체(하강 포함) 하는 동안, 반대편 팔은 라이브 조종 위치에
# 그대로 있으면 스테이션 프레임과 부딪힐 수 있음. 이를 막기 위해 반대편 팔을
# "NEUTRAL 자세에서 1번 모터만 다른 값으로 바꾼" 안전 위치로 잠깐 옮겨두고,
# 스왑이 끝나면 원래 있던 자리로 복귀시킴.

SAFE_RETREAT_MOTOR1_LEFT = 1749  # 왼팔 안전자세 1번 모터 tick
SAFE_RETREAT_MOTOR1_RIGHT = 3593  # 오른팔 안전자세 1번 모터 tick


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
# ===== 관절별 소프트 리밋 (가동범위 제한) =====
# =====================================================================
# 지금까지 tick 제한은 그리퍼(7번)의 NIPPER_SAFE_TICK_RANGE 하나뿐이었고,
# 1~6번 관절에는 가동범위 제한이 전혀 없었다. 마스터암 튐이나 조그 오프셋이
# 얹히면 관절이 기구 한계까지 그대로 밀고 들어가 서보가 과부하로 멈춘다.
#
# 형식: [ (min, max) 또는 None, ... ] — 모터 1~7 순서, 총 7개
#   None = 아직 미실측 → clamp 건너뜀 (지금 넣어도 동작이 바뀌지 않음)
#
# ⚠️ 측정값 그대로 넣지 말 것. joint_range_probe.py가 실측 한계에서
#    SAFETY_MARGIN(기본 60tick)만큼 안쪽으로 좁힌 값을 출력해준다.
#    한계에 딱 맞추면 백래시 때문에 결국 부딪힌다.
#
# ⚠️ 7번(그리퍼)은 여기 두지 말고 None으로 남길 것.
#    그리퍼는 MAX_OPEN/MAX_CLOSE와 clamp_for_nipper()가 따로 관리한다.

JOINT_LIMITS_LEFT = [
    (850, 2422),  # 1번 — 실측 850~1298, 동작자세 1003~2362 반영해 확장
    (956, 2043),  # 2번 — 실측 956~1299, 동작자세 1036~1983 반영해 확장
    (1013, 3278),  # 3번 — 실측 그대로
    (901, 2304),  # 4번 — 실측 901~1694, 동작자세 976~2244 반영해 확장
    (8, 3733),  # 5번 — 실측 그대로
    (968, 3015),  # 6번 — 실측 그대로
    None,  # 7번 그리퍼 — GRIPPER_SAFE_RANGE 로 별도 관리
]

JOINT_LIMITS_RIGHT = [
    (1452, 3653),  # 1번 — 실측 2734~3174, 동작자세 1512~3593 반영해 확장
    (868, 1830),  # 2번 — 실측 869~1830, 동작자세 928~1736 반영해 확장
    (760, 3664),  # 3번 — 실측 그대로
    (857, 2155),  # 4번 — 실측 857~1244, 동작자세 1017~2095 반영해 확장
    (15, 4000),  # 5번 — 실측 그대로
    (1149, 3005),  # 6번 — 실측 그대로
    None,  # 7번 그리퍼 — GRIPPER_SAFE_RANGE 로 별도 관리
]

# 서보 물리 한계 (소프트 리밋이 없는 축에도 최소한 이건 적용)
SERVO_TICK_MIN = 0
SERVO_TICK_MAX = 4095


def get_joint_limits(arm_side):
    """팔별 소프트 리밋 리스트 반환."""
    return JOINT_LIMITS_LEFT if arm_side == "left" else JOINT_LIMITS_RIGHT


def clamp_joint(arm_side, index, tick):
    """
    한 관절의 tick을 소프트 리밋 안으로 제한.
    해당 축이 미실측(None)이면 서보 물리 범위(0~4095)만 적용한다.
    """
    if tick is None:
        return None

    tick = int(tick)
    limits = get_joint_limits(arm_side)

    if 0 <= index < len(limits) and limits[index] is not None:
        lo, hi = limits[index]
        if lo is not None:
            tick = max(lo, tick)
        if hi is not None:
            tick = min(hi, tick)

    return max(SERVO_TICK_MIN, min(SERVO_TICK_MAX, tick))


def clamp_arm_ticks(arm_side, ticks):
    """
    팔 전체 tick 리스트에 소프트 리밋 적용.
    None 원소는 그대로 None으로 통과시킨다(호출부에서 판단).
    """
    return [clamp_joint(arm_side, i, t) for i, t in enumerate(ticks)]


# =====================================================================
# ===== 그리퍼별 안전범위 (조종 중 과조임/과개방 방지) =====
# =====================================================================
# gripper_dir_check.py 실측 끝단에서 GRIPPER_MARGIN만큼 안쪽으로 좁힌 값.
# None = 미실측 → 서보 물리범위(0~4095)만 적용, 즉 실질적으로 제한 없음.
#
# ⚠️ 반드시 MAX_OPEN/MAX_CLOSE 사이에 들어와야 한다.
#    안전범위가 실제 가동범위보다 넓으면 아무것도 막지 못한다
#    (기존 NIPPER_SAFE_TICK_RANGE=(1718,3593)이 정확히 그 상태였음).
#
# ⚠️ 이 clamp는 "라이브 조종" 경로에만 걸린다. 그리퍼 교체 시퀀스의
#    탈거/부착은 MAX_OPEN/MAX_CLOSE를 직접 쓰므로, 그쪽은 아래
#    MAX_OPEN/MAX_CLOSE 값 자체가 정확해야 안전하다.

GRIPPER_MARGIN = 30  # 실측 끝단에서 안쪽으로 좁힐 tick

GRIPPER_SAFE_RANGE = {
    ("left", "default"): (1202, 3739),  # 실측 1172~3769
    ("left", "vise"): (1434, 3971),  # 실측 1404~4001
    ("right", "fine"): (936, 3476),  # 실측  906~3506
    ("right", "nipper"): (1238, 3053),  # 실측 1208~3083
}


def clamp_gripper(arm_side, gripper_name, tick):
    """
    그리퍼(7번 모터) tick을 그 그리퍼의 안전범위로 제한.
    빈손(gripper_name=None)이거나 미실측이면 서보 물리범위만 적용한다.
    어떤 경우에도 예외를 던지지 않는다 — 그리퍼가 잠기는 일은 없어야 한다.
    """
    if tick is None:
        return None

    tick = int(tick)
    rng = GRIPPER_SAFE_RANGE.get((arm_side, gripper_name)) if gripper_name else None

    if rng is not None:
        lo, hi = rng
        if lo is not None:
            tick = max(lo, tick)
        if hi is not None:
            tick = min(hi, tick)

    return max(SERVO_TICK_MIN, min(SERVO_TICK_MAX, tick))


def clamp_for_nipper(gripper_tick):
    """
    (구버전 호환) 니퍼 전용 clamp.
    새 코드에서는 clamp_gripper('right', 'nipper', tick)를 쓸 것.
    """
    return clamp_gripper("right", "nipper", gripper_tick)


# =====================================================================
# ===== B안: 직접 전환(클리어런스) — 1↔2(오른팔), 3↔4(왼팔)만 해당 =====
# =====================================================================
# NEUTRAL을 거치지 않고, 그리퍼를 놓은(탈거한) 직후 바로 옆 스테이션 방향으로
# 팔을 이동시키는 "중간 경로".
#
# ===== 형식 (2026-08-22 변경: 순차이동 → 웨이포인트) =====
#
#   [(웨이포인트, 대기초), ...]                     ← 대기초 생략 가능
#   [(웨이포인트, 대기초, 이동시간초), ...]          ← 그 스텝만 속도 조절
#
#   웨이포인트 = [모터1, 모터2, 모터3, 모터4, 모터5, 모터6]  (6개 또는 7개)
#     - 숫자  → 그 모터를 그 tick으로 이동
#     - None  → 그 모터는 건드리지 않음 (직전 값 유지)
#     - 6개만 쓰면 7번(그리퍼)은 자동으로 유지됨
#       → 클리어런스 도중 그리퍼는 탈거 직후 MAX_OPEN 상태 그대로
#
# 한 웨이포인트 안의 모든 모터는 "동시에" 보간 이동한다.
# 모터 하나만 움직이고 싶으면 그 자리만 숫자로 쓰고 나머지는 None.
#
# ⚠️ 마지막에 "목표 스테이션 A값으로 이동"하는 스텝은 여기에 적지 않는다.
#    get_direct_swap_clearance()가 자동으로 붙여준다. (아래 함수 참고)
#
# ⚠️ 값을 비워두려면 빈 리스트 []로 남길 것. 그러면 호출부에서
#    "미실측" 경고를 띄우고 클리어런스 없이 바로 목표로 간다.

DIRECT_SWAP_PAIRS_RIGHT = {("fine", "nipper"), ("nipper", "fine")}
DIRECT_SWAP_PAIRS_LEFT = {("default", "vise"), ("vise", "default")}

DIRECT_SWAP_CLEARANCE_RIGHT = {
    # ----- 1번(미세) → 2번(니퍼) -----
    ("fine", "nipper"): [
        # 1) 전 축 동시 이동
        ([2241, 1166, 2907, 1976, 3198, 1763], 0.0),
        # 2) 1번 모터만
        ([1760, 1166, 2907, 1976, 3198, 1763], 0.0),
        # 3) 2번 모터만
        ([1760, 1736, 2907, 1976, 3198, 1763], 0.0),
        # 4) 전 축 동시 이동
        ([2070, 928, 3064, 2095, 2950, 1923], 0.0),
        # 5) 목표(니퍼) 스테이션 A값 → 자동으로 붙음
    ],
    # ----- 2번(니퍼) → 1번(미세) -----
    # TODO: 실측 후 위와 같은 형식으로 채우기 (보통 위 경로의 역순)
    ("nipper", "fine"): [
        # 1) 전 축 동시 이동
        ([2342, 1017, 3027, 1844, 3042, 1886], 0.0),
        # 2) 1번 모터만
        ([1512, 1017, 3027, 1844, 3042, 1886], 0.0),
        # 3) 2번 모터만
        ([1512, 1642, 3027, 1844, 3042, 1886], 0.0),
        # 4) 전 축 동시 이동
        ([1975, 1005, 3024, 1940, 2975, 1887], 0.0),
        # 5) 목표(니퍼) 스테이션 A값 → 자동으로 붙음
    ],
}

DIRECT_SWAP_CLEARANCE_LEFT = {
    # ----- 3번(기본) → 4번(바이스) -----
    # TODO: 실측 후 채우기
    ("default", "vise"): [
        # 1) 전 축 동시 이동
        ([1716, 1036, 1164, 1974, 1044, 1953], 0.0),
        # 2) 1번 모터만
        ([2362, 1036, 1164, 1974, 1044, 1953], 0.0),
        # 3) 2번 모터만
        ([2362, 1983, 1164, 1974, 1044, 1953], 0.0),
        # 4) 전 축 동시 이동
        ([2015, 1373, 1281, 2244, 705, 1950], 0.0),
        # 5) 목표(니퍼) 스테이션 A값 → 자동으로 붙음
    ],
    # ----- 4번(바이스) → 3번(기본) -----
    # TODO: 실측 후 채우기
    ("vise", "default"): [
        # 1) 전 축 동시 이동
        ([1826, 1040, 1182, 1948, 1035, 2069], 0.0),
        # 2) 1번 모터만
        ([2228, 1040, 1182, 1948, 1035, 2069], 0.0),
        # 3) 2번 모터만
        ([2228, 1895, 1182, 1948, 1035, 2069], 0.0),
        # 4) 전 축 동시 이동
        ([1789, 1088, 1168, 2054, 1045, 2016], 0.0),
        # 5) 목표(니퍼) 스테이션 A값 → 자동으로 붙음
    ],
}


# ===== Helper 함수들 =====


def is_direct_swap_pair(arm_side, held, target):
    """held/target 둘 다 있고, 1↔2 또는 3↔4 짝에 해당하면 True."""
    if held is None or target is None:
        return False
    pairs = DIRECT_SWAP_PAIRS_RIGHT if arm_side == "right" else DIRECT_SWAP_PAIRS_LEFT
    return (held, target) in pairs


def get_direct_swap_clearance(arm_side, held, target, include_target_pose=True):
    """
    직접 전환(클리어런스) 웨이포인트 시퀀스 가져오기.

    반환: [(웨이포인트, 대기초, 이동시간초), ...] 형태의 리스트 (또는 None)

    include_target_pose=True(기본)이면 마지막에 "목표 스테이션 A값" 웨이포인트를
    자동으로 덧붙인다. 스테이션 값을 재실측해도 여기만 고치면 되고, 클리어런스
    테이블에 같은 숫자를 두 번 적을 필요가 없다.

    ⚠️ 백래시 보정된 값(get_corrected_station_ticks)을 쓴다. 호출부(raspi2.py)가
       _attach_at()에 넘기는 target_ticks도 보정값이므로, 여기서 원본을 쓰면
       클리어런스 끝지점과 부착 시작점이 오프셋만큼 어긋나 점프가 생긴다.

    ⚠️ 그리퍼(7번)는 붙이지 않는다(6개만 씀) — 탈거 직후 MAX_OPEN 상태를
       유지한 채 목표 위치로 접근해야 하기 때문. _attach_at()이 그 자세에서
       그리퍼만 조여 부착한다.
    """
    table = (
        DIRECT_SWAP_CLEARANCE_RIGHT
        if arm_side == "right"
        else DIRECT_SWAP_CLEARANCE_LEFT
    )
    sequence = table.get((held, target))
    if not sequence:
        return None

    sequence = list(sequence)

    if include_target_pose:
        target_ticks = get_corrected_station_ticks(arm_side, target)
        if target_ticks is None:
            print(
                f"  [경고] {arm_side}/{target} 스테이션 tick 미실측 — "
                f"클리어런스 마지막 '목표 자세 이동' 스텝을 생략합니다"
            )
        else:
            sequence.append((list(target_ticks[:6]), 0.0))

    return sequence


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
#
# ⚠️ 스테이션 물리 위치를 옮긴 뒤라면 이 표는 다시 측정해야 유효함.
BACKLASH_OFFSET = {
    ("right", "fine"): [17, 0, -5, -13, 4, -2, 4],
    ("right", "nipper"): [21, 3, 3, -15, 4, -2, -2],
    ("left", "default"): [-16, 7, -4, -9, -2, -3, -3],
    ("left", "vise"): [0, -7, -4, -10, 3, -6, -3],
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
