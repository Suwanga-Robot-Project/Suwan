"""
한 팔에 대해 "지금 들고 있는 그리퍼를 내려놓고 새 그리퍼를 집는" 시퀀스.
양팔 공용으로 재사용 가능 — arm_side와 move_arm_to_fn을 외부에서 주입.

===== 두 가지 흐름 =====

[Flow A] 빈손 → 부착 (처음 시작할 때, 또는 짝이 아닌 조합)
  1. (리프트 하강 - FSM에서 처리)
  2. NEUTRAL(기본자세) 경유
  3. 목표 스테이션 위치로 이동, 이때 그리퍼는 "열린 상태"로 접근
  4. 그리퍼 "조이기" (부착)
  5. (리프트 상승 - FSM에서 처리)

[Flow B] 이미 착용 중 → 짝 전환 (1↔2 오른팔, 3↔4 왼팔만 해당, 클리어런스 경유)
  1. (리프트 하강 - FSM에서 처리, 그리퍼는 최대로 조인 상태 유지한 채 내려감)
  2. 현재 착용한 그리퍼의 스테이션 위치로 이동 (그리퍼 "최대로 조인 상태"로 꽂힘)
  3. 그리퍼 "벌리기" (탈거)
  4. 팔만 웨이포인트를 따라 이동 (클리어런스 경유, 리프트는 그대로 유지)
  5. 목표 스테이션 위치로 이동, 이때 그리퍼는 "열린 상태"로 접근
  6. 그리퍼 "조이기" (부착)

[짝이 아닌 조합 + 이미 착용 중] (예: 빈손 아닌데 니퍼→디폴트처럼 안 붙어있는 스테이션 간 이동)
  Flow A와 비슷하지만 먼저 현재 그리퍼를 NEUTRAL 경유해서 탈거하는 절차가 추가됨
  1. (리프트 하강)
  2. 현재 착용한 그리퍼의 스테이션 위치로 이동 (그리퍼 "최대로 조인 상태")
  3. 그리퍼 "벌리기" (탈거)
  4. NEUTRAL 경유
  5. 목표 스테이션 위치로 이동, 그리퍼 "열린 상태"로 접근
  6. 그리퍼 "조이기" (부착)
  7. (리프트 상승)
"""

import time
import station_positions

GRIPPER_MOTOR_INDEX = 6  # 7번째 모터(그리퍼) — 각 팔 tick 리스트의 마지막 인덱스
GRIPPER_SETTLE_SEC = 0.3  # 조임/펼침 사이 안정화 대기시간

# ===== 보간 이동 기본값 =====
GRADUAL_DURATION_SEC = 0.6  # 그리퍼 조임/펼침 등 "작은 동작" 이동 시간
GRADUAL_STEPS = 15  # 몇 단계로 나눠서 보간할지

# ===== 클리어런스(B안) 웨이포인트 전용 기본값 =====
# 여러 축이 동시에 수백 tick씩 움직이므로 그리퍼 개폐보다 더 느리고 촘촘하게 간다.
# 웨이포인트마다 개별로 덮어쓸 수 있음 (튜플 3번째 원소).
CLEARANCE_DURATION_SEC = 1.2
CLEARANCE_STEPS = 30

# UDP는 도착 보장이 없어서, 하필 "마지막 목표값" 패킷이 유실되면 라파가 직전
# 중간값에서 멈춰버린다. 클리어런스 마지막 웨이포인트는 곧바로 부착 동작으로
# 이어지는 자세라 특히 치명적이므로 몇 번 더 반복 전송한다.
# (raspi2.py의 _resend_final()과 같은 목적. move_arm_to_fn은 같은 값을 여러 번
#  보내도 부작용이 없으므로 유선/시뮬 백엔드에서도 안전하다.)
FINAL_RESEND_COUNT = 3
FINAL_RESEND_DELAY = 0.05


def _move_gradually(
    arm_side,
    from_ticks,
    to_ticks,
    move_arm_to_fn,
    duration=GRADUAL_DURATION_SEC,
    steps=GRADUAL_STEPS,
):
    """
    from_ticks에서 to_ticks까지 여러 스텝으로 나눠서 부드럽게 이동.
    탈거/부착/클리어런스 등 모든 내부 이동이 이 함수를 거쳐서 "천천히" 움직임.
    반환: to_ticks 그대로 (호출부 편의를 위해).
    """
    step_delay = duration / steps
    for step in range(1, steps + 1):
        ratio = step / steps
        interp = [
            int(round(s + (e - s) * ratio)) if (s is not None and e is not None) else e
            for s, e in zip(from_ticks, to_ticks)
        ]
        move_arm_to_fn(arm_side, interp)
        time.sleep(step_delay)
    return list(to_ticks)


def _detach_at(arm_side, gripper_name, station_ticks, move_arm_to_fn):
    """
    현재 착용 중인 gripper_name을 그 스테이션 위치에서 탈거.
    "최대로 조인 상태로 그 위치에 꽂힌 뒤" → "최대로 펼기" 순서.
    반환: 탈거 직후(개방된) tick 리스트.
    """
    gripper_max_close = station_positions.get_gripper_max_close(arm_side, gripper_name)
    gripper_max_open = station_positions.get_gripper_max_open(arm_side, gripper_name)

    if gripper_max_close is None or gripper_max_open is None:
        raise ValueError(
            f"{arm_side} {gripper_name}: MAX_CLOSE({gripper_max_close}) 또는 "
            f"MAX_OPEN({gripper_max_open})이 아직 실측되지 않았습니다"
        )

    # Step 1: 최대로 조인 상태로 그 위치에 꽂기
    close_ticks = list(station_ticks)
    close_ticks[GRIPPER_MOTOR_INDEX] = gripper_max_close
    print(
        f"  [탈거 1/2] {arm_side} {gripper_name}: 스테이션 위치로 천천히 이동 (그리퍼 MAX_CLOSE={gripper_max_close})"
    )
    _move_gradually(arm_side, station_ticks, close_ticks, move_arm_to_fn)
    time.sleep(GRIPPER_SETTLE_SEC)

    # Step 2: 최대로 펼기 (탈거됨)
    open_ticks = list(close_ticks)
    open_ticks[GRIPPER_MOTOR_INDEX] = gripper_max_open
    print(
        f"  [탈거 2/2] {arm_side} {gripper_name}: 그리퍼 천천히 벌리기 (MAX_OPEN={gripper_max_open}) → 빠짐!"
    )
    _move_gradually(arm_side, close_ticks, open_ticks, move_arm_to_fn)
    time.sleep(GRIPPER_SETTLE_SEC)

    return open_ticks


# =====================================================================
# ===== 클리어런스(B안) 웨이포인트 이동 =====
# =====================================================================


def _resolve_waypoint(current_ticks, waypoint):
    """
    웨이포인트를 "실제로 보낼 7개짜리 tick 리스트"로 펼친다.

    - waypoint 안의 None은 "이 모터는 건드리지 않음" → 직전 값 유지
    - waypoint가 6개뿐이면 7번째(그리퍼)는 자동으로 직전 값 유지
      → 클리어런스 도중 그리퍼는 탈거 직후의 MAX_OPEN 상태 그대로 유지됨

    덕분에 "1번 모터만 움직임" 같은 스텝을
        ([1760, None, None, None, None, None], 0.0)
    처럼 한 줄로 쓸 수 있고, 나머지 값을 복사해 적을 필요가 없다.
    """
    resolved = list(current_ticks)
    for i, value in enumerate(waypoint):
        if i >= len(resolved):
            break  # 7개를 넘는 값은 무시
        if value is not None:
            resolved[i] = value
    return resolved


def _unpack_waypoint_entry(entry):
    """(waypoint,) / (waypoint, wait) / (waypoint, wait, duration) 전부 허용."""
    waypoint = entry[0]
    wait_after = entry[1] if len(entry) > 1 and entry[1] is not None else 0.0
    duration = (
        entry[2] if len(entry) > 2 and entry[2] is not None else CLEARANCE_DURATION_SEC
    )
    return waypoint, wait_after, duration


def _move_waypoints(arm_side, current_ticks, waypoints, move_arm_to_fn):
    """
    클리어런스 웨이포인트 시퀀스를 순서대로 실행.

    waypoints: [(웨이포인트, 대기초, 이동시간초), ...]
      웨이포인트 = [모터1, ..., 모터6] 또는 [모터1, ..., 모터7]
                   각 원소는 목표 tick 또는 None(그대로 유지)

    각 웨이포인트까지 _move_gradually()로 보간 이동한 뒤 wait_after만큼 정지.
    반환: 시퀀스가 끝난 뒤의 tick 리스트.
    """
    working_ticks = list(current_ticks)

    for idx, entry in enumerate(waypoints, start=1):
        waypoint, wait_after, duration = _unpack_waypoint_entry(entry)
        target_ticks = _resolve_waypoint(working_ticks, waypoint)

        if any(t is None for t in target_ticks):
            print(
                f"  [경고] 클리어런스 웨이포인트 {idx}에 미실측(None) 값이 남아 있습니다 — 건너뜀"
            )
            continue

        moved = [
            f"{i + 1}번:{b}"
            for i, (a, b) in enumerate(zip(working_ticks, target_ticks))
            if a != b
        ]
        if not moved:
            print(f"  [클리어런스 {idx}/{len(waypoints)}] 변화 없음 — 건너뜀")
            continue

        print(
            f"  [클리어런스 {idx}/{len(waypoints)}] {', '.join(moved)} "
            f"({duration}초에 걸쳐 천천히)"
        )
        working_ticks = _move_gradually(
            arm_side,
            working_ticks,
            target_ticks,
            move_arm_to_fn,
            duration=duration,
            steps=CLEARANCE_STEPS,
        )
        if wait_after > 0:
            print(f"    → 도달, {wait_after}초 대기")
            time.sleep(wait_after)

    # 마지막 자세 재전송 (UDP 유실 대비)
    for _ in range(FINAL_RESEND_COUNT):
        move_arm_to_fn(arm_side, working_ticks)
        time.sleep(FINAL_RESEND_DELAY)

    return working_ticks


def _move_sequential_legacy(arm_side, current_ticks, sequence, move_arm_to_fn):
    """
    (구 형식) 한 번에 모터 하나씩만 움직이는 순차이동.
    sequence: [(모터_인덱스, 목표tick, 이번스텝후_대기초), ...]
    새 코드에서는 웨이포인트 형식을 쓰지만, 예전 데이터가 남아 있어도
    깨지지 않도록 남겨둠.
    """
    working_ticks = list(current_ticks)
    for motor_index, target_tick, wait_after in sequence:
        if target_tick is None:
            print(
                f"  [경고] 클리어런스 시퀀스에 미실측 값이 있습니다 (모터 인덱스 {motor_index}) — 건너뜀"
            )
            continue

        next_ticks = list(working_ticks)
        next_ticks[motor_index] = target_tick
        print(f"  [클리어런스] 모터{motor_index + 1}번을 {target_tick}로 천천히 이동")
        working_ticks = _move_gradually(
            arm_side, working_ticks, next_ticks, move_arm_to_fn
        )
        print(f"    → 도달, {wait_after}초 대기")
        time.sleep(wait_after)
    return working_ticks


def _move_sequential(arm_side, current_ticks, sequence, move_arm_to_fn):
    """
    클리어런스 이동 진입점. 두 가지 형식을 자동 판별한다.

    [신] 웨이포인트 형식: [([tick, tick, ...], 대기초), ...]
         → 여러 축 동시 이동 + 단일 축 이동을 한 형식으로 표현
    [구] 순차 형식:       [(모터_인덱스, 목표tick, 대기초), ...]

    raspi2.py 등 호출부는 이 함수 이름/인자를 그대로 쓰면 되므로 수정 불필요.
    """
    if not sequence:
        return list(current_ticks)

    first_element = sequence[0][0]
    if isinstance(first_element, (list, tuple)):
        return _move_waypoints(arm_side, current_ticks, sequence, move_arm_to_fn)
    return _move_sequential_legacy(arm_side, current_ticks, sequence, move_arm_to_fn)


def _attach_at(arm_side, gripper_name, station_ticks, move_arm_to_fn):
    """
    target station_ticks 위치로 그리퍼를 "연 상태로 접근" → "조이기(부착)".
    반환: 부착 직후(조여진) tick 리스트.

    ⚠️ 이 함수는 팔이 이미 station_ticks 자세에 도달해 있다고 가정한다.
       (Flow B에서는 클리어런스 마지막 웨이포인트가 목표 스테이션 A값이므로 충족됨)
    """
    gripper_max_close = station_positions.get_gripper_max_close(arm_side, gripper_name)
    gripper_max_open = station_positions.get_gripper_max_open(arm_side, gripper_name)

    if gripper_max_close is None or gripper_max_open is None:
        raise ValueError(
            f"{arm_side} {gripper_name}: MAX_CLOSE({gripper_max_close}) 또는 "
            f"MAX_OPEN({gripper_max_open})이 아직 실측되지 않았습니다"
        )

    # Step 1: 그리퍼 "연 상태"로 목표 위치까지 접근
    open_ticks = list(station_ticks)
    open_ticks[GRIPPER_MOTOR_INDEX] = gripper_max_open
    print(
        f"  [부착 1/2] {arm_side} {gripper_name}: 목표 위치로 천천히 접근 (그리퍼 MAX_OPEN={gripper_max_open})"
    )
    _move_gradually(arm_side, station_ticks, open_ticks, move_arm_to_fn)
    time.sleep(GRIPPER_SETTLE_SEC)

    # Step 2: 그리퍼 조이기 (부착)
    close_ticks = list(open_ticks)
    close_ticks[GRIPPER_MOTOR_INDEX] = gripper_max_close
    print(
        f"  [부착 2/2] {arm_side} {gripper_name}: 그리퍼 천천히 조이기 (MAX_CLOSE={gripper_max_close}) → 부착!"
    )
    _move_gradually(arm_side, open_ticks, close_ticks, move_arm_to_fn)
    time.sleep(GRIPPER_SETTLE_SEC)

    return close_ticks


def swap_gripper(arm_side, held_gripper, target_gripper, move_arm_to_fn):
    """
    arm_side: 'left' 또는 'right'
    held_gripper: 현재 들고 있는 그리퍼 이름(없으면 None, = 빈손)
    target_gripper: 바꾸려는 그리퍼 이름(전체탈거면 None)
    move_arm_to_fn: 팔을 tick 배열로 이동시키는 함수 (외부에서 주입)
    """
    neutral_ticks = (
        station_positions.NEUTRAL_TICKS_LEFT
        if arm_side == "left"
        else station_positions.NEUTRAL_TICKS_RIGHT
    )

    def go_neutral():
        if all(t is not None for t in neutral_ticks):
            print(f"  → NEUTRAL 자세로 이동 ({arm_side}팔)")
            move_arm_to_fn(arm_side, neutral_ticks)
        else:
            print(f"  [경고] NEUTRAL tick이 None인 상태 — 건너뜀")

    use_direct_swap = station_positions.is_direct_swap_pair(
        arm_side, held_gripper, target_gripper
    )

    # =====================================================================
    # [Flow B] 짝 전환 (클리어런스 경유, NEUTRAL 안 거침) — 1↔2, 3↔4만 해당
    # =====================================================================
    if use_direct_swap:
        print(
            f"\n📍 [{arm_side} 팔] Flow B (짝 전환, 클리어런스): {held_gripper} → {target_gripper}"
        )

        held_ticks = station_positions.get_corrected_station_ticks(
            arm_side, held_gripper
        )
        if held_ticks is None:
            raise ValueError(
                f"{arm_side}/{held_gripper} tick 값이 아직 실측되지 않았습니다"
            )

        # 1) 현재 착용한 그리퍼의 스테이션 위치로 (최대조임 상태로 꽂힘) → 벌리기 (탈거)
        after_detach_ticks = _detach_at(
            arm_side, held_gripper, held_ticks, move_arm_to_fn
        )

        # 2) 웨이포인트를 따라 이동 (클리어런스, 리프트는 그대로)
        #    마지막 웨이포인트는 get_direct_swap_clearance()가 목표 스테이션
        #    A값으로 자동으로 붙여줌 → _attach_at 진입 시 점프가 생기지 않음
        clearance_sequence = station_positions.get_direct_swap_clearance(
            arm_side, held_gripper, target_gripper
        )
        if clearance_sequence:
            print(f"  → 클리어런스 웨이포인트 이동 시작")
            _move_sequential(
                arm_side, after_detach_ticks, clearance_sequence, move_arm_to_fn
            )
        else:
            print(
                f"  [경고] {arm_side} {held_gripper}->{target_gripper} 클리어런스 시퀀스 "
                f"미실측 — 클리어런스 단계 없이 바로 목표로 이동합니다"
            )

        # 3) 목표 스테이션 위치로 (열린 상태로 접근) → 조이기 (부착)
        target_ticks = station_positions.get_corrected_station_ticks(
            arm_side, target_gripper
        )
        if target_ticks is None:
            raise ValueError(
                f"{arm_side}/{target_gripper} tick 값이 아직 실측되지 않았습니다"
            )
        _attach_at(arm_side, target_gripper, target_ticks, move_arm_to_fn)

    # =====================================================================
    # [Flow A] 빈손→부착 또는 짝이 아닌 조합 (NEUTRAL 경유)
    # =====================================================================
    else:
        print(
            f"\n📍 [{arm_side} 팔] Flow A (NEUTRAL 경유): {held_gripper} → {target_gripper}"
        )

        if held_gripper is not None:
            # 이미 뭔가 들고 있으면 먼저 그 스테이션 위치에서 탈거
            held_ticks = station_positions.get_corrected_station_ticks(
                arm_side, held_gripper
            )
            if held_ticks is None:
                raise ValueError(
                    f"{arm_side}/{held_gripper} tick 값이 아직 실측되지 않았습니다"
                )
            _detach_at(arm_side, held_gripper, held_ticks, move_arm_to_fn)
            go_neutral()
        else:
            # 빈손이면 바로 NEUTRAL(기본자세)로
            go_neutral()

        if target_gripper is not None:
            # 목표 스테이션 위치로 (열린 상태로 접근) → 조이기 (부착)
            target_ticks = station_positions.get_corrected_station_ticks(
                arm_side, target_gripper
            )
            if target_ticks is None:
                raise ValueError(
                    f"{arm_side}/{target_gripper} tick 값이 아직 실측되지 않았습니다"
                )
            _attach_at(arm_side, target_gripper, target_ticks, move_arm_to_fn)

    return target_gripper  # 새로 보유하게 된 그리퍼 (None이면 빈손)
