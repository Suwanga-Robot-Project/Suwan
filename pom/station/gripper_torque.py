"""
gripper_torque_limit.py
─────────────────────────────────────────────────────────────
그리퍼 서보의 토크 리밋을 확인하고 설정한다.

왜 필요한가
    물체를 물면 그리퍼는 목표 tick까지 못 간다. 그런데 명령은 계속
    그 tick으로 나가므로, 서보는 "오차가 안 줄어든다"고 판단해서
    최대 토크를 계속 쓴다. 기어가 못 버티고 미끄러지는 게 "드드득" 소리다.

    토크 리밋을 걸면 서보가 그 이상 힘을 못 쓴다. 파이썬이 무엇을
    명령하든, 코드가 죽든 상관없이 서보 내부에서 걸린다.

    tick 범위 제한(clamp)으로는 이 문제를 못 막는다. 명령값이 범위 안에
    있어도 물체가 막고 있으면 똑같이 갈리기 때문이다.

토크 리밋 레지스터
    주소 48, 2바이트, 값 범위 0~1000 (1000 = 100%)
    SRAM이므로 전원을 끄면 초기화된다 → 프로그램 시작 시마다 써야 한다.
    EEPROM이 아니므로 잘못 써도 되돌리기 쉽다.

사용법
    python gripper_torque_limit.py                 # 현재 값 읽기만
    python gripper_torque_limit.py 400             # 400으로 설정
    python gripper_torque_limit.py 400 --test      # 설정 후 조임 테스트

    ⚠ 파일 위쪽 PORT / GRIPPER_ID 를 대상에 맞게 고칠 것
      왼팔  COM12 / ID 7      오른팔  COM14 / ID 15
"""

import sys
import time

from scservo_sdk import *  # noqa: F401,F403

# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전 확인
# ═══════════════════════════════════════════════════════════

PORT = "COM12"
GRIPPER_ID = 7

BAUDRATE = 1000000
PROTOCOL_END = 0

# 조임 테스트용 목표 (--test 옵션에서만 사용)
TEST_CLOSE_TICK = 3739  # 미세 그리퍼 안전범위 상한
TEST_OPEN_TICK = 1202
TEST_HOLD_SEC = 5.0

# ═══════════════════════════════════════════════════════════

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60

LOAD_MASK = 0x03FF
TORQUE_LIMIT_MAX = 1000


def swap16(v):
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


class Gripper:
    def __init__(self, ph, port):
        self.ph = ph
        self.port = port

    def _read2(self, addr):
        v, c, e = self.ph.read2ByteTxRx(self.port, GRIPPER_ID, addr)
        return None if (c != 0 or e != 0) else v

    def pos(self):
        v = self._read2(ADDR_PRESENT_POSITION)
        if v is None:
            return None
        if 0 <= v <= 4095:
            return v
        sw = swap16(v)
        return sw if 0 <= sw <= 4095 else None

    def load(self):
        v = self._read2(ADDR_PRESENT_LOAD)
        return (v & LOAD_MASK) if v is not None else None

    def torque_limit(self):
        v = self._read2(ADDR_TORQUE_LIMIT)
        if v is None:
            return None
        if 0 <= v <= TORQUE_LIMIT_MAX:
            return v
        sw = swap16(v)
        return sw if 0 <= sw <= TORQUE_LIMIT_MAX else None

    def set_torque_limit(self, value):
        value = max(0, min(TORQUE_LIMIT_MAX, int(value)))
        self.ph.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_TORQUE_LIMIT, value)
        time.sleep(0.1)
        return self.torque_limit()

    def torque(self, on):
        self.ph.write1ByteTxRx(
            self.port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1 if on else 0
        )

    def goto(self, tick):
        self.ph.write2ByteTxRx(
            self.port, GRIPPER_ID, ADDR_GOAL_POSITION, max(0, min(4095, int(tick)))
        )


def run_test(g, target_limit):
    """조임 테스트 — 물체를 물린 채로 실행할 것."""
    print()
    print("─" * 60)
    print("  조임 테스트")
    print("─" * 60)
    print("  그리퍼 사이에 실제로 잡을 물체를 끼워두세요.")
    print(f"  {TEST_CLOSE_TICK}까지 조이라고 명령하고 {TEST_HOLD_SEC}초 유지합니다.")
    print("  드드득 소리가 나는지 귀로 확인하세요.")
    input("\n  준비되면 Enter > ")

    g.goto(TEST_OPEN_TICK)
    time.sleep(1.5)

    g.goto(TEST_CLOSE_TICK)
    t0 = time.time()
    max_load = 0
    positions = []

    while time.time() - t0 < TEST_HOLD_SEC:
        p = g.pos()
        ld = g.load()
        if p is not None:
            positions.append(p)
        if ld is not None:
            max_load = max(max_load, ld)
        print(
            f"    경과 {time.time()-t0:4.1f}s   위치 {p}   부하 {ld}   "
            f"목표까지 {TEST_CLOSE_TICK - p if p else '?'}   ",
            end="\r",
        )
        time.sleep(0.1)

    print(" " * 78, end="\r")
    g.goto(TEST_OPEN_TICK)
    time.sleep(1.0)

    if not positions:
        print("  [실패] 위치를 못 읽었습니다")
        return

    drift = max(positions) - min(positions)
    print(f"\n  최대 부하        {max_load}")
    print(f"  멈춘 위치        {positions[-1]}  (목표 {TEST_CLOSE_TICK})")
    print(f"  유지 중 위치변동 {drift} tick")
    print()
    if drift > 30:
        print("  ⚠ 위치가 계속 변했습니다 = 기어가 밀리고 있습니다.")
        print(
            f"     토크 리밋을 더 낮추세요 (현재 {target_limit} → {int(target_limit*0.7)} 시도)"
        )
    else:
        print("  ▶ 위치가 안정적입니다. 이 토크 리밋에서는 밀리지 않습니다.")
        print("     물체를 놓칠 정도로 약하면 조금씩 올리세요.")


def main():
    args = [a for a in sys.argv[1:]]
    do_test = "--test" in args
    numeric = [a for a in args if a.isdigit()]
    new_limit = int(numeric[0]) if numeric else None

    port = PortHandler(PORT)
    ph = PacketHandler(PROTOCOL_END)

    if not port.openPort():
        sys.exit(f"[에러] {PORT} 열기 실패 — N5.py 등을 종료했는지 확인하세요")
    if not port.setBaudRate(BAUDRATE):
        sys.exit("[에러] 통신속도 설정 실패")

    g = Gripper(ph, port)

    try:
        print()
        print("═" * 60)
        print(f"  그리퍼 토크 리밋  —  ID {GRIPPER_ID} @ {PORT}")
        print("═" * 60)

        pos = g.pos()
        if pos is None:
            print("  [에러] 위치를 못 읽었습니다. GRIPPER_ID / PORT 를 확인하세요.")
            return

        current = g.torque_limit()
        print(f"  현재 위치       {pos}")
        print(f"  현재 부하       {g.load()}")
        print(f"  현재 토크 리밋  {current}  (최대 {TORQUE_LIMIT_MAX})")

        if current is None:
            print()
            print("  ⚠ 토크 리밋을 읽지 못했습니다.")
            print("     이 서보 모델은 주소 48이 토크 리밋이 아닐 수 있습니다.")
            print("     데이터시트를 확인하고 ADDR_TORQUE_LIMIT 을 고치세요.")
            return

        if new_limit is None:
            print()
            print("  값을 바꾸려면 인자로 숫자를 주세요:")
            print("      python gripper_torque_limit.py 400")
            print()
            print("  권장 시작값")
            print("      300~400  물체를 잡되 밀리지 않는 선 (여기서 시작)")
            print("      500~600  좀 더 세게 — 무거운 물체용")
            print("      1000     제한 없음 (지금 상태, 드드득의 원인)")
            return

        g.torque(True)
        time.sleep(0.2)
        applied = g.set_torque_limit(new_limit)
        print()
        print(f"  ▶ 토크 리밋 {current} → {applied} 로 설정")

        if applied != new_limit:
            print(f"     [주의] 요청값 {new_limit} 과 다릅니다. 레지스터를 확인하세요.")

        print()
        print("  ⚠ 이 값은 SRAM이라 전원을 끄면 초기화됩니다.")
        print("     프로그램 시작 시마다 다시 써야 유지됩니다 (아래 코드 참고).")

        if do_test:
            run_test(g, applied)

    except KeyboardInterrupt:
        print("\n\n  [중단] 사용자 종료")
    finally:
        port.closePort()
        print("\n  포트 닫음")


if __name__ == "__main__":
    main()
