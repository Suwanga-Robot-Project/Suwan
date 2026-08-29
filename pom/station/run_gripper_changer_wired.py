"""
[라파 없이, 노트북-로봇팔 유선직결 테스트용]
그리퍼교체 새 순서(2026-08-07 재설계: 먼저 정렬→그 다음 하강)를
실제 팔(COM12/14 서보)로 테스트하되, 리프트(상하이동)는 라파 없이
엔터 입력으로 "리미트스위치 눌림"을 대신하는 방식.

===== 이 스크립트가 쓰는 연결 =====
COM13: 실제 키캡 읽기 (STM32 ADC/키캡 패킷)
COM12: 왼팔 서보 (servo_control.py 경유)
COM14: 오른팔 서보 (servo_control.py 경유)
리프트: 진짜 GPIO 없음 — lift_control_sim.py로 대체 (엔터 누르면 "하강 완료"로 침)

===== 순서 (Nexus_5.py와 동일 로직) =====
공통: 원래 위치 저장 → NEUTRAL로 천천히 이동 → 4초 대기(흔들림 방지)
빈손: 목표 스테이션 위치로 천천히 이동(아직 위) → 하강(엔터) → 부착 → 상승(엔터) → 복귀
보유중: 보유 스테이션 위치로 천천히 이동(아직 위) → 하강(엔터) → 탈거
       → B안 클리어런스 순차이동 → 목표 부착 → 상승(엔터) → 복귀

⚠️ 현재 팔 위치를 실시간으로 추적할 라이브 조종 루프가 없으므로,
   "원래 위치"는 NEUTRAL에서 시작해서 매 스왑이 끝날 때마다 갱신되는 값을 씁니다.
   (실제 조종 중 위치로 정확히 복귀하는 건 Nexus_5.py의 역할이고, 이 스크립트는
   "새 순서/타이밍/하드웨어 동작"만 확인하는 용도입니다.)
"""

import time
import serial
import struct

import station_positions
import arm_swap_sequence
import key_input_handler
import servo_control
import lift_control_sim as lift_backend

# ===== 키캡 시리얼 설정 =====
PORT_KEYCAP = "COM13"
BAUD_KEYCAP = 115200

PACKET_HEADER = b"\xaa\x55"
PACKET_SIZE = 52
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBBH")
KEY_STATES_INDEX = 26  # unpacked 튜플에서 key_states 위치


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


def parse_keys(key_states_byte):
    """
    key_states 1바이트에서 1~5번 키 눌림 여부(bool) 추출.
    ⚠️ main.c 펌웨어에서 이미 반전 처리되어 있어(비트=1이면 눌림),
    여기서는 추가로 뒤집지 않음(정상 semantics 그대로 사용).
    """
    key1 = bool(key_states_byte & (1 << 0))
    key2 = bool(key_states_byte & (1 << 1))
    key3 = bool(key_states_byte & (1 << 2))
    key4 = bool(key_states_byte & (1 << 3))
    key5 = bool(key_states_byte & (1 << 4))
    return key1, key2, key3, key4, key5


# ===== 현재 팔 위치 추적 (라이브 조종 루프가 없어서 NEUTRAL로 시작, 스왑마다 갱신) =====
current_left_ticks = list(station_positions.NEUTRAL_TICKS_LEFT)
current_right_ticks = list(station_positions.NEUTRAL_TICKS_RIGHT)

GRIPPER_HELD_LEFT = None
GRIPPER_HELD_RIGHT = None

# ===== 타이밍 파라미터 (Nexus_5.py와 동일) =====
NEUTRAL_TRANSITION_SECONDS = 2.0
NEUTRAL_TRANSITION_STEPS = 40
NEUTRAL_WAIT_SECONDS = 2.0
STATION_APPROACH_SECONDS = 2.0
STATION_APPROACH_STEPS = 40


def move_arm_to(arm_side, ticks):
    """실제 서보로 직접 write (COM12/14), 현재위치 추적 배열도 같이 갱신."""
    global current_left_ticks, current_right_ticks

    target_array = current_left_ticks if arm_side == "left" else current_right_ticks
    for i, t in enumerate(ticks):
        if t is not None:
            target_array[i] = int(t)

    tick_str = " ".join(f"{t:5d}" if t is not None else "  ?" for t in ticks)
    print(f"      [서보 이동] {arm_side}: {tick_str}")

    servo_control.move_arm_to(arm_side, ticks)


def _lerp_ticks(start_ticks, end_ticks, ratio):
    result = []
    for s, e in zip(start_ticks, end_ticks):
        if s is None or e is None:
            result.append(e)
        else:
            result.append(int(round(s + (e - s) * ratio)))
    return result


def move_arm_gradually(arm_side, target_ticks, duration_seconds, steps=40):
    """(기존 방식, 지금은 안 씀 — 참고용) 7개 모터 전부 동시에 보간 이동."""
    current = list(current_left_ticks if arm_side == "left" else current_right_ticks)
    step_delay = duration_seconds / steps
    for step in range(1, steps + 1):
        ratio = step / steps
        interp = _lerp_ticks(current, target_ticks, ratio)
        move_arm_to(arm_side, interp)
        time.sleep(step_delay)


# ===== 모터별 순차이동 파라미터 =====
MOTOR_BY_MOTOR_DURATION = 0.3  # 모터 하나 이동에 걸리는 시간(초)
MOTOR_BY_MOTOR_STEPS = 10  # 모터 하나 이동을 몇 단계로 나눌지


def move_arm_motor_by_motor(
    arm_side,
    target_ticks,
    duration_per_motor=MOTOR_BY_MOTOR_DURATION,
    steps_per_motor=MOTOR_BY_MOTOR_STEPS,
):
    """현재 위치에서 target_ticks까지, 모터를 1번부터 7번까지 순서대로 하나씩 이동."""
    working = list(current_left_ticks if arm_side == "left" else current_right_ticks)
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


def run_gripper_swap(arm_side, target_gripper):
    """
    한 팔의 그리퍼 교체 전체 시퀀스 (블로킹).
    Nexus_5.py의 run_gripper_swap()과 동일한 순서, 다만 UDP 대신 실제 서보 write.
    """
    global GRIPPER_HELD_LEFT, GRIPPER_HELD_RIGHT

    other_side = "right" if arm_side == "left" else "left"

    held = GRIPPER_HELD_LEFT if arm_side == "left" else GRIPPER_HELD_RIGHT
    saved_ticks = list(
        current_left_ticks if arm_side == "left" else current_right_ticks
    )
    other_saved_ticks = list(
        current_right_ticks if arm_side == "left" else current_left_ticks
    )
    neutral_ticks = (
        station_positions.NEUTRAL_TICKS_LEFT
        if arm_side == "left"
        else station_positions.NEUTRAL_TICKS_RIGHT
    )

    print(f"\n>>> [{arm_side} 팔] 그리퍼교체 시작: {held} → {target_gripper}")
    print(f"    (원래 위치 저장: {saved_ticks})")

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
        target_ticks = station_positions.get_corrected_station_ticks(
            arm_side, target_gripper
        )
        if target_ticks is None:
            raise ValueError(
                f"{arm_side}/{target_gripper} tick 값이 아직 실측되지 않았습니다"
            )

        print(f"    → 목표({target_gripper}) 스테이션 위치로 천천히 이동 (아직 위)")
        move_arm_motor_by_motor(arm_side, target_ticks)

        elapsed = lift_backend.descend_until_bottom_switch()  # ← 여기서 엔터 대기
        time.sleep(1.0)

        arm_swap_sequence._attach_at(
            arm_side, target_gripper, target_ticks, move_arm_to
        )
        new_held = target_gripper

    else:
        held_ticks = station_positions.get_corrected_station_ticks(arm_side, held)
        if held_ticks is None:
            raise ValueError(f"{arm_side}/{held} tick 값이 아직 실측되지 않았습니다")

        print(f"    → 보유 중인({held}) 스테이션 위치로 천천히 이동 (아직 위)")
        move_arm_motor_by_motor(arm_side, held_ticks)

        elapsed = lift_backend.descend_until_bottom_switch()  # ← 여기서 엔터 대기
        time.sleep(1.0)

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

    lift_backend.ascend_full(elapsed)  # ← 여기서도 잠깐 대기(엔터 아님, 시간만큼 sleep)

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


def main():
    try:
        ser = serial.Serial(PORT_KEYCAP, BAUD_KEYCAP, timeout=1)
    except Exception as e:
        print(f"시리얼 열기 실패({PORT_KEYCAP}):", e)
        return

    # ===== 서보 포트(COM12/14) 열기 + 토크 ON — 이걸 먼저 해야 move_arm_to()가 동작함 =====
    try:
        servo_control.init_servos()
    except Exception as e:
        print(f"서보 초기화 실패: {e}")
        ser.close()
        return

    # ===== 실제 현재 위치 읽어오기 — 이게 없으면 "가정한 위치(NEUTRAL)"에서 시작해서
    #       첫 이동이 보간 없이 한번에 점프하는 문제가 생김 =====
    global current_left_ticks, current_right_ticks
    print(">>> 서보 현재 위치 읽는 중...")
    read_left = servo_control.read_current_ticks("left")
    read_right = servo_control.read_current_ticks("right")

    if all(t is not None for t in read_left):
        current_left_ticks = read_left
        print(f"    왼팔 현재 위치: {read_left}")
    else:
        print(
            f"    [경고] 왼팔 위치 읽기 일부 실패 — NEUTRAL로 대체 (첫 이동이 급격할 수 있음)"
        )

    if all(t is not None for t in read_right):
        current_right_ticks = read_right
        print(f"    오른팔 현재 위치: {read_right}")
    else:
        print(
            f"    [경고] 오른팔 위치 읽기 일부 실패 — NEUTRAL로 대체 (첫 이동이 급격할 수 있음)"
        )

    print(f"{PORT_KEYCAP} 열기 성공. 키캡 감지 시작 (Ctrl+C로 종료)")
    print(">>> 리프트는 엔터 입력으로 시뮬레이션됩니다 (라파 없음)\n")

    prev_keys = (False, False, False, False, False)
    buf = bytearray()

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
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
                unpacked = PACKET_STRUCT.unpack(raw_packet)

                crc_calc = calc_crc16_ccitt(raw_packet[: PACKET_SIZE - 2])
                crc_received = unpacked[-1]
                if crc_calc != crc_received:
                    del buf[:2]
                    continue

                key_states_byte = unpacked[KEY_STATES_INDEX]
                current_keys = parse_keys(key_states_byte)

                edge_keys = tuple(
                    now and not prev for now, prev in zip(current_keys, prev_keys)
                )
                prev_keys = current_keys

                if any(edge_keys):
                    e1, e2, e3, e4, e5 = edge_keys
                    left_target, right_target = key_input_handler.parse_key_input(
                        e1, e2, e3, e4, e5
                    )

                    if left_target is not None:
                        actual = (
                            None
                            if left_target == key_input_handler.DROP_ALL
                            else left_target
                        )
                        if actual != GRIPPER_HELD_LEFT:
                            run_gripper_swap("left", actual)

                    if right_target is not None:
                        actual = (
                            None
                            if right_target == key_input_handler.DROP_ALL
                            else right_target
                        )
                        if actual != GRIPPER_HELD_RIGHT:
                            run_gripper_swap("right", actual)

                del buf[:PACKET_SIZE]

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n=== 종료 ===")


if __name__ == "__main__":
    main()
