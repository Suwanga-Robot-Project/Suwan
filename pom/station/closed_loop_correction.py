"""
클로즈드루프(실시간 재조정) 이동 모듈.

서보의 백래시/유격 오차가 "매번 다르게" 나는 경우, 고정된 보정표로는
대응이 안 됨 — 대신 이동시킨 뒤 실제 위치를 읽어서, 오차만큼 다시
밀어주는 걸 반복해서 목표에 수렴시키는 방식.

⚠️ 실측 결과 반영(2026-08-09): 오차만큼 "그대로" 반대로 밀면 오히려
   오버슈트(반대 방향으로 과하게 밀림)가 나거나, 작은 보정은 백래시
   구간(dead zone) 안에서 흡수되어 전혀 반응 없는 경우가 확인됨.
   → correction_gain(기본 0.7, 감쇠)으로 오버슈트 방지 +
     stuck_boost(기본 1.6, 막힘 감지 시 세게 밀기)로 dead zone 돌파.

다른 파일(move_to_station.py, position_accuracy_diagnostic.py,
나중에 Nexus_5.py 등)에서 이렇게 가져다 씀:

    import closed_loop_correction as clc

    def move_fn(ticks):
        move_arm(portHandler_left, packetHandler_left, MOTORS_LEFT, ticks)

    def read_fn():
        return read_current_ticks(portHandler_left, packetHandler_left, MOTORS_LEFT)

    final_ticks, success = clc.move_with_correction(
        target_ticks=[1437, 1017, 1272, 1584, 1862, 2115, 2644],
        move_fn=move_fn,
        read_fn=read_fn,
    )

move_fn/read_fn을 이렇게 "그 파일에 맞는 실제 구현"으로 감싸서 넘겨주기만
하면, 이 모듈 자체는 어떤 파일이든 그대로 재사용 가능합니다
(COM12/14 유선이든, 나중에 UDP+라파 피드백이든 상관없음).
"""

import time


def move_with_pid_correction(
    target_ticks,
    move_fn,
    read_fn,
    kp=0.6,
    ki=0.15,
    tolerance=8,
    max_retries=12,
    settle_delay=0.4,
    max_deviation_from_target=150,
    integral_clamp=200,
    verbose=True,
):
    """
    PID(정확히는 PI) 제어 기반 클로즈드루프 이동.

    기존 move_with_correction()의 "오차*배율 + 막히면 세게 밀기" 방식은
    제가 즉흥적으로 짠 휴리스틱이라 오버슈트/진동이 남아있었음.
    이건 그 대신 표준 PID 제어를 씀:

        보정량 = Kp × 오차 + Ki × (오차의 누적합)

    - P(비례)항: 오차가 클수록 크게 보정 (기존 gain과 비슷)
    - I(적분)항: 오차가 계속 남아있으면(막혀서 안 없어지면) 시간이 지날수록
                 저절로 누적되어 점점 크게 밀어붙임 — "막힘 감지" 같은
                 특수 로직을 따로 안 짜도, 수학적으로 자연스럽게 뚫림
    - integral_clamp: 적분항이 무한정 커지지 않게 막는 안전장치(anti-windup)
    - max_deviation_from_target: 명령값이 목표에서 이 이상 못 벗어나게 제한
                                  (기존과 동일한 안전장치)

    Args:
        target_ticks: 목표 tick 리스트
        move_fn(ticks): 이동 함수
        read_fn() -> ticks: 실제 위치 읽기 함수
        kp: 비례 계수 (오차에 곧바로 반응하는 정도, 기본 0.6)
        ki: 적분 계수 (오차가 오래 남아있을 때 점점 세게 미는 정도, 기본 0.15)
        tolerance: 이 오차 이내면 성공
        max_retries: 최대 시도 횟수
        settle_delay: 이동 후 대기시간
        max_deviation_from_target: 안전장치 — 목표에서 최대 이탈 허용범위
        integral_clamp: 적분항 상한(anti-windup)
        verbose: 로그 출력 여부

    Returns:
        (final_ticks, success)
    """
    current_command = list(target_ticks)
    integral = [0.0] * len(target_ticks)

    for attempt in range(1, max_retries + 1):
        move_fn(current_command)
        time.sleep(settle_delay)
        actual = read_fn()

        if actual is None or any(a is None for a in actual):
            if verbose:
                print(f"  [PID 시도 {attempt}] 위치 읽기 실패 — 중단")
            return current_command, False

        errors = [
            (a - t) if (a is not None and t is not None) else 0
            for a, t in zip(actual, target_ticks)
        ]
        max_abs_error = max(abs(e) for e in errors)

        if verbose:
            error_str = " ".join(f"{e:+d}" for e in errors)
            integral_str = " ".join(f"{i:+.0f}" for i in integral)
            print(
                f"  [PID 시도 {attempt}] 오차: [{error_str}]  누적(I): [{integral_str}]  (최대오차 {max_abs_error}, 허용 {tolerance})"
            )

        if max_abs_error <= tolerance:
            if verbose:
                print(f"  [PID 완료] {attempt}번 만에 허용오차 이내 도달")
            return current_command, True

        next_command = []
        for i, (t, e) in enumerate(zip(target_ticks, errors)):
            if t is None:
                next_command.append(None)
                continue

            integral[i] += e
            integral[i] = max(
                -integral_clamp, min(integral_clamp, integral[i])
            )  # anti-windup

            correction = kp * e + ki * integral[i]
            proposed = t - correction

            # 안전장치: 목표에서 너무 멀리 못 벗어나게
            min_allowed = t - max_deviation_from_target
            max_allowed = t + max_deviation_from_target
            next_tick = int(round(max(min_allowed, min(max_allowed, proposed))))
            next_command.append(next_tick)

        current_command = next_command

    if verbose:
        print(
            f"  [PID 실패] {max_retries}번 시도 후에도 오차가 허용범위 초과 — 마지막 값으로 종료"
        )
    return current_command, False


def move_with_correction(
    target_ticks,
    move_fn,
    read_fn,
    tolerance=8,
    max_retries=10,
    settle_delay=0.4,
    correction_gain=0.7,
    stuck_kick_base=15,
    stuck_kick_escalation=1.6,
    stuck_kick_max=80,
    stuck_threshold=3,
    max_deviation_from_target=150,
    max_saturated_stuck_before_abort=2,
    verbose=True,
):
    """
    목표 tick으로 이동시키고, 실제 도달값을 확인해서 오차만큼 재보정을
    반복하는 클로즈드루프 이동.

    ⚠️ 안전장치(2026-08-09 추가, 실기 테스트 중 위험 상황 발견 후):
       킥이 최대치(stuck_kick_max)까지 커져도 계속 반응이 없으면, 이건
       백래시가 아니라 진짜 물리적 한계(하드 리밋/기구적 걸림)일 가능성이
       높음 — 이 경우 계속 세게 미는 건 오히려 위험함(모터 과열, 갑자기
       저항이 풀렸을 때 팔이 크게 튈 위험). 그래서:
       1) 명령값이 원래 목표에서 max_deviation_from_target 이상 벗어나지
          못하게 강제로 제한함
       2) 최대킥 상태로 max_saturated_stuck_before_abort번 연속 막히면,
          남은 재시도를 다 안 쓰고 즉시 중단하고 명확히 경고함

    Args:
        target_ticks: 진짜로 원하는 목표 tick 리스트
        move_fn(ticks): 팔을 그 tick으로 이동시키는 함수
        read_fn() -> ticks: 실제 현재 위치를 읽는 함수 (읽기 실패 모터는 None)
        tolerance: 이 오차(tick) 이내면 "도달 성공"으로 판정
        max_retries: 최대 몇 번까지 재시도할지
        settle_delay: 이동 후 실제 위치 읽기 전 대기시간(초)
        correction_gain: (막히지 않은 정상 상태) 오차 보정 배율 — 오버슈트 방지용 감쇠
        stuck_kick_base: 막힘 처음 감지됐을 때, 직전 명령에서 절대적으로 몇 tick을 더 밀지
        stuck_kick_escalation: 같은 모터가 연속으로 막히면 킥 크기를 이 배율로 키움
        stuck_kick_max: 킥 크기 상한
        stuck_threshold: 이전 시도 대비 오차 변화가 이 값보다 작으면 "막혔다"
        max_deviation_from_target: 명령값이 원래 목표에서 이 tick 이상
                                    벗어나지 못하게 강제 제한 (안전장치)
        max_saturated_stuck_before_abort: 킥이 최대치에 도달한 채로 이만큼
                                           연속 막히면 즉시 중단 (안전장치)
        verbose: 시도할 때마다 로그 출력할지

    Returns:
        (final_ticks, success)
    """
    current_command = list(target_ticks)
    prev_errors = None
    consecutive_stuck_count = [0] * len(target_ticks)
    saturated_stuck_count = [0] * len(target_ticks)  # 최대킥 상태로 연속 몇 번 막혔는지

    for attempt in range(1, max_retries + 1):
        move_fn(current_command)
        time.sleep(settle_delay)
        actual = read_fn()

        if actual is None or any(a is None for a in actual):
            if verbose:
                print(
                    f"  [보정 시도 {attempt}] 위치 읽기 실패 — 보정 중단, 마지막 명령값 유지"
                )
            return current_command, False

        errors = [
            (a - t) if (a is not None and t is not None) else 0
            for a, t in zip(actual, target_ticks)
        ]
        max_abs_error = max(abs(e) for e in errors)

        if verbose:
            error_str = " ".join(f"{e:+d}" for e in errors)
            print(
                f"  [보정 시도 {attempt}] 오차: [{error_str}]  (최대 {max_abs_error}, 허용 {tolerance})"
            )

        if max_abs_error <= tolerance:
            if verbose:
                print(f"  [보정 완료] {attempt}번 만에 허용오차 이내 도달")
            return current_command, True

        if prev_errors is not None and errors == prev_errors:
            if verbose:
                print(
                    f"    [경고] 이전 시도와 오차가 전체 채널 완전히 동일 — 통신 문제 또는 물리적 한계(기구적 걸림/하드리밋) 의심"
                )

        # ===== 다음 명령 계산 =====
        next_command = []
        stuck_motors = []
        abort_now = False

        for i, (t, e) in enumerate(zip(target_ticks, errors)):
            if t is None:
                next_command.append(None)
                continue

            is_stuck = (
                prev_errors is not None
                and abs(e - prev_errors[i]) < stuck_threshold
                and abs(e) > tolerance
            )

            if is_stuck:
                consecutive_stuck_count[i] += 1
                kick = stuck_kick_base * (
                    stuck_kick_escalation ** (consecutive_stuck_count[i] - 1)
                )
                kick_saturated = kick >= stuck_kick_max
                kick = min(kick, stuck_kick_max)

                if kick_saturated:
                    saturated_stuck_count[i] += 1
                    if saturated_stuck_count[i] >= max_saturated_stuck_before_abort:
                        abort_now = True
                else:
                    saturated_stuck_count[i] = 0

                direction = -1 if e > 0 else 1
                proposed_tick = current_command[i] + direction * kick

                # ===== 안전장치: 원래 목표에서 너무 멀리 못 벗어나게 제한 =====
                min_allowed = t - max_deviation_from_target
                max_allowed = t + max_deviation_from_target
                next_tick = int(
                    round(max(min_allowed, min(max_allowed, proposed_tick)))
                )

                stuck_motors.append((i + 1, round(kick, 1)))
            else:
                consecutive_stuck_count[i] = 0
                saturated_stuck_count[i] = 0
                next_tick = int(round(t - e * correction_gain))

            next_command.append(next_tick)

        if verbose and stuck_motors:
            motor_str = ", ".join(f"모터{m}번(킥{k})" for m, k in stuck_motors)
            print(f"    [막힘 감지] {motor_str} — 직전 명령에서 더 크게 밀기")

        if abort_now:
            if verbose:
                stuck_final = [
                    i + 1
                    for i, c in enumerate(saturated_stuck_count)
                    if c >= max_saturated_stuck_before_abort
                ]
                print(
                    f"  [안전 중단] 모터{stuck_final}이 최대 세기로도 "
                    f"{max_saturated_stuck_before_abort}번 연속 무반응 — 백래시가 아니라 "
                    f"물리적 한계(하드리밋/기구적 걸림)로 판단, 더 밀지 않고 즉시 중단합니다.\n"
                    f"  [필요 조치] 실제 배선/기구 걸림/서보 토크 한계를 직접 점검하세요"
                    f" (소프트웨어로는 더 진행 안 함 — 안전을 위해)"
                )
            return current_command, False

        prev_errors = errors
        current_command = next_command

    if verbose:
        print(
            f"  [보정 실패] {max_retries}번 시도 후에도 오차가 허용범위 초과 — 마지막 값으로 종료"
        )
        stuck_final = [i + 1 for i, c in enumerate(consecutive_stuck_count) if c >= 2]
        if stuck_final:
            print(
                f"    [진단] 모터{stuck_final} 계속 막혀있었음 — 배선/기구적 걸림/하드 리밋 등"
                f" 물리적 원인 점검 권장 (알고리즘으로는 더 이상 해결 어려움)"
            )
    return current_command, False


if __name__ == "__main__":
    print("=== closed_loop_correction.py 자체 테스트 (오버슈트+막힘 시뮬레이션) ===")

    import random

    fake_actual_position = [0] * 7
    # 백래시 폭(모터별) — 이 범위 안의 작은 이동은 실제로 반영이 잘 안 됨
    backlash_width = [12, 6, 4, 15, 3, 4, 5]
    last_commanded = [None] * 7

    def fake_move_fn(ticks):
        global fake_actual_position, last_commanded
        new_actual = []
        for i, tick in enumerate(ticks):
            if last_commanded[i] is None:
                # 첫 이동은 백래시만큼 밀려서 도착
                new_actual.append(tick + backlash_width[i])
            else:
                delta = tick - last_commanded[i]
                if abs(delta) < backlash_width[i]:
                    # 작은 보정은 백래시 구간에 흡수되어 거의 안 움직임
                    new_actual.append(fake_actual_position[i])
                else:
                    new_actual.append(
                        tick + (backlash_width[i] if delta > 0 else -backlash_width[i])
                    )
        fake_actual_position = new_actual
        last_commanded = list(ticks)
        print(f"    (테스트) 명령: {ticks} → 실제 도착 가정: {fake_actual_position}")

    def fake_read_fn():
        return fake_actual_position

    target = [1000, 1000, 1000, 1000, 1000, 1000, 1000]
    final, ok = move_with_correction(
        target, fake_move_fn, fake_read_fn, tolerance=5, max_retries=8
    )
    print(f"\n최종 명령값: {final}, 성공: {ok}")
