"""
[단독 테스트용] 오른팔만 하드코딩된 NEUTRAL_TICKS_RIGHT로 천천히 이동시키는 스크립트.
왼팔은 NEUTRAL_TICKS_LEFT로 고정, 움직이지 않음.

라파(라즈베리파이)에 UDP로 직접 전송 — Nexus_5.py(raspi2.py)와 동일한 22개 필드
패킷 포맷을 그대로 사용하되, 팔 외의 나머지(팬틸트/리프트/바퀴/키캡)는 전부 중립값
고정으로 보내서 아무 영향도 안 주도록 함.

⚠️ 팔이 라파에 물려있어서 이 스크립트는 실제 현재 위치를 직접 읽을 수 없습니다.
   그래서 START_TICKS_RIGHT를 "마지막으로 확인하신 값"으로 하드코딩해뒀습니다.
   실제로 그 자리에 없다면(그 사이에 조종해서 위치가 바뀌었다면), 첫 이동이
   급격하게 튈 수 있으니 실행 직전 실제 위치와 맞는지 확인하세요.
"""

import socket
import time

# ===== 라파 UDP 설정 (Nexus_5.py와 동일) =====
RPI_IP = "192.168.1.104"
RPI_PORT = 5005

# ===== 목표: NEUTRAL 자세 =====
NEUTRAL_TICKS_LEFT = [1003, 1112, 2142, 976, 1858, 1939, 2034]
NEUTRAL_TICKS_RIGHT = [2983, 1044, 2020, 1017, 2102, 2088, 1966]

# ===== 오른팔 시작 위치 — 방금 관찰하신 마지막 값으로 설정 =====
# (라파에 물린 서보라 직접 못 읽어서, 마지막으로 확인하신 값을 그대로 넣었습니다.
#  실행 직전 실제 자세랑 다르면 이 값을 먼저 고쳐주세요.)
START_TICKS_RIGHT = [1411, 1905, 1707, 1095, 2579, 2043, 3755]

# ===== 모터별 순차이동 파라미터 =====
MOTOR_BY_MOTOR_DURATION = 0.3  # 모터 하나 이동에 걸리는 시간(초)
MOTOR_BY_MOTOR_STEPS = 10  # 몇 단계로 나눠서 보간할지

# ===== UDP 유실 대비 재전송 =====
RESEND_COUNT = 5
RESEND_DELAY = 0.05

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_packet(left_ticks, right_ticks):
    """
    Nexus_5.py와 동일한 22개 필드 포맷으로 UDP 전송.
    팔(왼쪽7+오른쪽7) 외의 나머지(팬틸트/리프트/바퀴/키캡/노브)는
    전부 중립/비활성값으로 고정해서, 이 스크립트가 팔 이동 말고는
    아무 것도 건드리지 않도록 함.
    """
    udp_data = (
        "<"
        + ",".join(str(int(t)) for t in left_ticks)
        + ","
        + ",".join(str(int(t)) for t in right_ticks)
        + ",511,511,0"  # pan, tilt, lift_state — 전부 중립
        + ",0,2033,2033"  # sw1_toggle, 바퀴용 ind2/ind3 — 중립(정지)
        + ",000000"  # key_states_str — 아무 키도 안 눌림(6자리)
        + ",0>"  # sw_toggle(오른쪽 노브)
    )
    try:
        udp_sock.sendto(udp_data.encode("utf-8"), (RPI_IP, RPI_PORT))
    except Exception as e:
        print(f"UDP 전송 오류: {e}")


def resend_final(left_ticks, right_ticks):
    """UDP 유실 대비 — 최종값을 여러 번 반복 전송."""
    for _ in range(RESEND_COUNT):
        send_packet(left_ticks, right_ticks)
        time.sleep(RESEND_DELAY)


def move_right_motor_by_motor(
    from_ticks,
    to_ticks,
    duration_per_motor=MOTOR_BY_MOTOR_DURATION,
    steps_per_motor=MOTOR_BY_MOTOR_STEPS,
):
    """
    오른팔만 모터 1번(=서보ID 9)부터 순서대로 하나씩 이동.
    왼팔은 매 프레임 NEUTRAL_TICKS_LEFT로 고정해서 같이 전송(안 움직임).
    """
    working = list(from_ticks)
    for idx in range(len(working)):
        start_tick = working[idx]
        target_tick = to_ticks[idx]
        motor_id = idx + 9  # 오른팔 서보ID는 9~15

        if start_tick == target_tick:
            print(f"  모터{motor_id}번: 이미 목표값과 같음 — 건너뜀")
            continue

        print(f"  모터{motor_id}번 이동 시작: {start_tick} → {target_tick}")
        step_delay = duration_per_motor / steps_per_motor
        for step in range(1, steps_per_motor + 1):
            ratio = step / steps_per_motor
            working[idx] = int(round(start_tick + (target_tick - start_tick) * ratio))
            send_packet(NEUTRAL_TICKS_LEFT, working)
            time.sleep(step_delay)
        working[idx] = target_tick
        print(f"  모터{motor_id}번 도착")

    return working


def main():
    print(">>> 왼팔은 NEUTRAL 고정, 오른팔만 천천히 NEUTRAL로 이동합니다.")
    print(f"    오른팔 시작값: {START_TICKS_RIGHT}")
    print(f"    오른팔 목표값: {NEUTRAL_TICKS_RIGHT}")
    print(f"    라파 주소: {RPI_IP}:{RPI_PORT}\n")

    # 시작 전, 왼팔 NEUTRAL + 오른팔 시작값을 먼저 몇 번 보내서 자세 확정
    print(">>> 시작 위치 확정 전송 중...")
    resend_final(NEUTRAL_TICKS_LEFT, START_TICKS_RIGHT)
    time.sleep(0.5)

    print(">>> 오른팔 모터 하나씩 순서대로 이동 시작\n")
    final = move_right_motor_by_motor(START_TICKS_RIGHT, NEUTRAL_TICKS_RIGHT)

    print("\n>>> 최종값 재전송 중 (UDP 유실 대비)...")
    resend_final(NEUTRAL_TICKS_LEFT, final)

    print(">>> 완료 — 오른팔이 NEUTRAL 자세로 도착했습니다.")


if __name__ == "__main__":
    main()
