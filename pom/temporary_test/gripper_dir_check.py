"""
gripper_dir_check.py
─────────────────────────────────────────────────────────────
그리퍼 개폐 방향과 실제 tick 범위를 서보 구동으로 자동 측정한다.

손으로 그리퍼를 못 움직이는 경우(감속비가 커서 역구동이 안 되는 경우)를
위해, 서보가 직접 천천히 밀면서 "더 이상 안 가는 지점"을 찾아낸다.

안전장치
    · 한 번에 STEP_TICK(10)씩만 이동
    · 부하가 ABORT_LOAD 를 넘으면 즉시 중단
    · 목표에 못 미치는 상태가 연속되면 끝단으로 판정하고 정지
    · 총 이동량이 MAX_TRAVEL 을 넘으면 중단
    · 어떤 경우에도 마지막에 시작 위치로 복귀

⚠ 실행 전에 그리퍼 사이에 물체가 없는지 확인할 것.

사용법
    1. 아래 PORT / GRIPPER_ID / NAME 수정
    2. COM 포트를 쓰는 프로그램(N5.py 등) 전부 종료
    3. python gripper_dir_check.py

    대상            PORT     GRIPPER_ID   NAME
    왼팔 디폴트     COM12         7        default
    왼팔 바이스     COM12         7        vise
    오른팔 미세     COM14        15        fine
    오른팔 니퍼     COM14        15        nipper
"""

import inspect
import sys
import time

from scservo_sdk import *  # noqa: F401,F403
import scservo_sdk as _sdk

# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전 확인
# ═══════════════════════════════════════════════════════════

PORT = "COM12"
GRIPPER_ID = 7
NAME = "default"

BAUDRATE = 1000000
PROTOCOL_END = 0

# ═══════════════════════════════════════════════════════════
#  안전 파라미터
# ═══════════════════════════════════════════════════════════

STEP_TICK = 10  # 한 스텝 이동량
STEP_DELAY = 0.08  # 스텝 간 대기 (서보가 따라올 시간)
ABORT_LOAD = 450  # 이 부하(0~1023)를 넘으면 즉시 중단
STALL_MARGIN = 25  # 목표 대비 이만큼 못 가면 정체로 간주
STALL_FRAMES = 3  # 연속 몇 프레임 정체하면 끝단으로 판정
MAX_TRAVEL = 2600  # 한 방향 최대 이동량 (폭주 방지)
PROBE_TICK = 150  # 방향 확인용 시험 이동량

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
POS_MIN, POS_MAX = 0, 4095


def _make_packet_handler(port):
    attempts = []
    for n in ("sms_sts", "SMS_STS", "sts", "scscl"):
        cls = getattr(_sdk, n, None)
        if inspect.isclass(cls):
            attempts.append((lambda c=cls: c(port), n))
    f = getattr(_sdk, "PacketHandler", None)
    if f is not None:
        attempts.append((lambda: f(PROTOCOL_END), f"PacketHandler({PROTOCOL_END})"))
        attempts.append((lambda: f(port, PROTOCOL_END), "PacketHandler(port,end)"))
    cls = getattr(_sdk, "protocol_packet_handler", None)
    if inspect.isclass(cls):
        attempts.append((lambda: cls(port, PROTOCOL_END), "protocol(port,end)"))
        attempts.append((lambda: cls(PROTOCOL_END), "protocol(end)"))
    for make, name in attempts:
        try:
            ph = make()
        except Exception:
            continue
        if hasattr(ph, "read2ByteTxRx") and hasattr(ph, "write1ByteTxRx"):
            return ph, name
    return None, None


def swap16(v):
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


class Gripper:
    def __init__(self, ph, port):
        self.ph = ph
        self.port = port

    def pos(self):
        v, c, e = self.ph.read2ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_POSITION)
        if c != 0 or e != 0:
            return None
        if POS_MIN <= v <= POS_MAX:
            return v
        sw = swap16(v)
        return sw if POS_MIN <= sw <= POS_MAX else None

    def load(self):
        v, c, e = self.ph.read2ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_LOAD)
        return (v & 0x03FF) if (c == 0 and e == 0) else 0

    def torque(self, on):
        self.ph.write1ByteTxRx(
            self.port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1 if on else 0
        )

    def goto(self, tick):
        t = max(POS_MIN, min(POS_MAX, int(tick)))
        self.ph.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_POSITION, t)

    def move_to(self, target, settle=0.5):
        """한 번에 이동 (짧은 거리용)."""
        self.goto(target)
        time.sleep(settle)
        return self.pos()

    def creep(self, direction, label):
        """방향(+1/-1)으로 천천히 밀면서 끝단을 찾는다."""
        start = self.pos()
        if start is None:
            return None, "위치 읽기 실패"

        tick = start
        stall = 0
        traveled = 0
        reason = "최대 이동량 도달"

        while traveled < MAX_TRAVEL:
            tick += STEP_TICK * direction
            if not (POS_MIN <= tick <= POS_MAX):
                tick = max(POS_MIN, min(POS_MAX, tick))
                reason = "서보 범위 한계"
                self.goto(tick)
                time.sleep(STEP_DELAY)
                break

            self.goto(tick)
            time.sleep(STEP_DELAY)

            actual = self.pos()
            ld = self.load()
            if actual is None:
                continue

            traveled = abs(actual - start)
            print(
                f"    {label}  목표 {tick:4d}  실제 {actual:4d}  " f"부하 {ld:4d}",
                end="\r",
            )

            if ld >= ABORT_LOAD:
                reason = f"부하 한계 {ld}"
                break

            if abs(actual - tick) > STALL_MARGIN:
                stall += 1
                if stall >= STALL_FRAMES:
                    reason = "기계적 끝단"
                    break
            else:
                stall = 0

        print(" " * 60, end="\r")
        final = self.pos()
        # 끝단에 힘주고 있지 않도록 살짝 물러남
        if final is not None:
            self.goto(final - STEP_TICK * direction)
            time.sleep(0.3)
            final = self.pos()
        return final, reason


def main():
    print()
    print("═" * 60)
    print(f"  그리퍼 범위 자동 측정 — {NAME}  (ID {GRIPPER_ID} @ {PORT})")
    print("═" * 60)
    print("\n  ⚠ 그리퍼 사이에 물체가 없는지 확인하세요.")
    print("     서보가 직접 천천히 움직여서 양쪽 끝을 찾습니다.")

    port = PortHandler(PORT)
    ph, sdk_name = _make_packet_handler(port)
    if ph is None:
        sys.exit("[에러] 패킷 핸들러 생성 실패")

    try:
        port.openPort()
        port.setBaudRate(BAUDRATE)
    except Exception as e:
        print(f"\n[에러] {PORT} 열기 실패: {e}")
        print("       N5.py / 시리얼 모니터를 모두 종료했는지 확인하세요")
        return

    g = Gripper(ph, port)
    print(f"  SDK 방식: {sdk_name}")

    start = None
    try:
        g.torque(True)
        time.sleep(0.3)
        start = g.pos()
        if start is None:
            print("\n  [에러] 위치를 못 읽었습니다. GRIPPER_ID / PORT 확인")
            return
        print(f"  현재 위치: {start}   부하: {g.load()}")

        input("\n  준비되면 Enter (Ctrl+C로 중단): ")

        # ── 1. 방향 확인 ─────────────────────────────────
        print(f"\n  [1/3] tick을 {PROBE_TICK} 올려봅니다...")
        g.move_to(start + PROBE_TICK, settle=1.0)
        after = g.pos()
        print(f"        {start} → {after}")

        ans = ""
        while ans not in ("o", "c"):
            ans = (
                input(
                    "\n        방금 그리퍼가 어떻게 됐나요?\n"
                    "          o = 벌어짐 (열림)\n"
                    "          c = 오므라듦 (조임)\n"
                    "        입력: "
                )
                .strip()
                .lower()
            )

        g.move_to(start, settle=1.0)

        # tick 증가 = 열림(o) 이면, 조임은 -1 방향
        close_dir = -1 if ans == "o" else +1
        open_dir = -close_dir

        # ── 2. 완전 개방 끝단 ────────────────────────────
        print(f"\n  [2/3] 완전히 벌리는 중...")
        opened, r1 = g.creep(open_dir, "개방")
        print(f"        완전 개방 tick = {opened}   ({r1})")

        # ── 3. 완전 조임 끝단 ────────────────────────────
        print(f"\n  [3/3] 완전히 오므리는 중...")
        closed, r2 = g.creep(close_dir, "조임")
        print(f"        완전 조임 tick = {closed}   ({r2})")

        # ── 결과 ─────────────────────────────────────────
        print()
        print("─" * 60)
        print("  결과")
        print("─" * 60)

        if opened is None or closed is None:
            print("  위치를 읽지 못해 결과를 낼 수 없습니다")
            return

        span = abs(closed - opened)
        if span < 100:
            print(f"  ⚠ 개방과 조임의 차이가 {span}뿐입니다.")
            print("     ABORT_LOAD 를 올리거나 물체가 끼어있지 않은지 확인하세요.")
            return

        if closed > opened:
            print(f"  ▶ 조이면 tick이 커집니다   ({opened} → {closed})")
        else:
            print(f"  ▶ 조이면 tick이 작아집니다  ({opened} → {closed})")
        print(f"     전체 가동폭 {span} tick")

        print(f"\n  ① gripper_contact.py 의 GRIPPER_RANGE 에 입력:")
        print(f'         "{NAME}": ({opened}, {closed}),      # (개방, 조임)')

        print(f"\n  ② station_positions.py 값과 비교해보세요:")
        print(f"         {NAME.upper()}_MAX_OPEN  = {opened}")
        print(f"         {NAME.upper()}_MAX_CLOSE = {closed}")
        print(f"     기존 값과 다르면 그리퍼가 옮겨졌거나 재조립된 것입니다.")
        print()

    except KeyboardInterrupt:
        print("\n\n  [중단] 사용자 종료")
    finally:
        try:
            if start is not None:
                print("  시작 위치로 복귀 중...")
                g.goto(start)
                time.sleep(1.0)
            g.torque(False)
        finally:
            port.closePort()
        print("  토크 해제 및 포트 닫기 완료")


if __name__ == "__main__":
    main()
