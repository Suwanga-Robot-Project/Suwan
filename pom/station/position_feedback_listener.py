"""
[노트북에서 실행] 라파가 되돌려주는 실제 서보 위치를 받아서 콘솔에 출력.

⚠️ 중요: 라파가 tick을 쓰자마자 바로 읽어서 보내기 때문에, 스트리밍되는 값들은
   "아직 이동 중인 순간의 스냅샷"일 수 있습니다. 값이 계속 바뀌다가 1초 이상
   더 이상 안 바뀌면 그때를 "정착값(진짜 최종 위치)"으로 표시해줍니다.
   → 비교할 땐 반드시 [정착값 확정] 표시가 붙은 값만 쓰세요.

사용법:
  1) 이 스크립트 실행 (별도 터미널)
  2) Nexus_5.py 실행해서 그리퍼교체 테스트
  3) [정착값 확정]이 뜬 값을 Nexus_5.py 콘솔의 "보정 적용된 목표"랑 비교
"""

import socket
import time

FEEDBACK_PORT = 5010
SOCKET_TIMEOUT_SEC = 0.2  # 정착 감지를 위해 짧게 (Ctrl+C 반응성도 좋아짐
SETTLE_SEC = 1.0  # 이 시간 동안 값이 안 바뀌면 "정착됨"으로 판정

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", FEEDBACK_PORT))
sock.settimeout(SOCKET_TIMEOUT_SEC)

print(f">>> 라파 위치 피드백 대기 중... (포트 {FEEDBACK_PORT})")
print(">>> Ctrl+C로 종료")
print(">>> (라파 쪽에 피드백 코드가 아직 안 붙여져 있으면 계속 조용한 게 정상입니다)\n")

last_values = None
last_change_time = None
settled_announced = True  # 처음엔 아직 값이 없으니 정착 알림 스킵

running = True
while running:
    try:
        data, addr = sock.recvfrom(1024)
    except socket.timeout:
        # 데이터가 한동안 안 왔어도, 마지막 값이 정착 조건을 만족하는지 체크
        if (
            last_values is not None
            and not settled_announced
            and last_change_time is not None
            and time.time() - last_change_time >= SETTLE_SEC
        ):
            left_str = " ".join(f"{v:>5s}" for v in last_values[0:7])
            right_str = " ".join(f"{v:>5s}" for v in last_values[7:14])
            print(
                f"[정착값 확정] L: {left_str}  |  R: {right_str}   ← 이 값으로 비교하세요\n"
            )
            settled_announced = True
        continue
    except KeyboardInterrupt:
        print("\n=== 종료 ===")
        running = False
        continue

    try:
        text = data.decode("utf-8").strip()

        if not (text.startswith("<") and text.endswith(">")):
            continue

        values = text[1:-1].split(",")
        if len(values) != 14:
            print(f"[경고] 예상과 다른 필드 개수: {len(values)}개 — {text}")
            continue

        if values != last_values:
            left_str = " ".join(f"{v:>5s}" for v in values[0:7])
            right_str = " ".join(f"{v:>5s}" for v in values[7:14])
            print(f"[실시간] L: {left_str}  |  R: {right_str}")
            last_values = values
            last_change_time = time.time()
            settled_announced = False

    except KeyboardInterrupt:
        print("\n=== 종료 ===")
        running = False

sock.close()
