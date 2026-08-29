"""
gripper_load_probe.py
─────────────────────────────────────────────────────────────
그리퍼 부하 프로파일 실측 스크립트

내일 학교에서 이거 하나만 실행하면 임계값이 나온다.
그리퍼 모터 하나만 살아 있으면 되므로, 다른 모터가 고장 나 있어도
독립적으로 수행 가능하다.

동작:
    스크립트가 그리퍼를 MAX_OPEN 에서 MAX_CLOSE 방향으로
    조금씩(STEP_TICK) 닫으면서 매 스텝 Present Load 를 읽어 기록한다.
    시나리오 5개를 순서대로 안내하고, 끝나면 권장 임계값을 계산한다.

안전장치:
    - ABORT_LOAD 초과 시 즉시 중단하고 개방으로 복귀
    - 각 시나리오 종료 시 항상 MAX_OPEN 으로 복귀
    - Ctrl+C 시에도 개방 복귀 후 종료

사용법:
    python gripper_load_probe.py
"""

import csv
import inspect
import sys
import time
from datetime import datetime

from scservo_sdk import *  # noqa: F401,F403
import scservo_sdk as _sdk

PROTOCOL_END = 0


# STS/SMS 계열은 0

SDK_NAME = None


def swap16(v):
    """상하위 바이트를 뒤집는다 (엔디안 불일치 보정)."""
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def _make_packet_handler(port):
    """설치된 SDK에 맞는 패킷 핸들러를 만들어 반환한다."""
    attempts = []

    for n in ("sms_sts", "SMS_STS", "smsSts", "sts", "STS", "scscl", "SCSCL"):
        cls = getattr(_sdk, n, None)
        if inspect.isclass(cls):
            attempts.append((lambda c=cls: c(port), n))

    f = getattr(_sdk, "PacketHandler", None)
    if f is not None:
        attempts.append((lambda: f(PROTOCOL_END), f"PacketHandler({PROTOCOL_END})"))
        attempts.append((lambda: f(port, PROTOCOL_END), "PacketHandler(port,end)"))

    cls = getattr(_sdk, "protocol_packet_handler", None)
    if inspect.isclass(cls):
        attempts.append(
            (lambda: cls(port, PROTOCOL_END), "protocol_packet_handler(port,end)")
        )
        attempts.append((lambda: cls(PROTOCOL_END), "protocol_packet_handler(end)"))

    for make, name in attempts:
        try:
            ph = make()
        except Exception:
            continue
        if hasattr(ph, "read2ByteTxRx") and hasattr(ph, "write1ByteTxRx"):
            return ph, name

    return None, None


# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전에 여기만 확인
# ═══════════════════════════════════════════════════════════

DEVICENAME = "COM14"  # 왼팔이면 "COM12" / 라파면 "/dev/ttyACM1"
GRIPPER_ID = 15  # 왼팔 7, 오른팔 15
GRIPPER_NAME = "fine"  # default / vise / fine / nipper

BAUDRATE = 1000000

# ═══════════════════════════════════════════════════════════

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_LOAD = 60

LOAD_MAGNITUDE_MASK = 0x03FF

GRIPPER_RANGE = {
    "default": (1749, 3254),
    "vise": (1478, 4095),
    "fine": (2292, 3742),
    "nipper": (2048, 3268),
}

STEP_TICK = 8  # 한 스텝에 움직일 tick
STEP_DELAY = 0.06  # 스텝 사이 대기 (서보가 따라올 시간)
ABORT_LOAD = 700  # 이 부하를 넘으면 즉시 중단 (0~1023)
SETTLE_DELAY = 0.5  # 개방 복귀 후 안정화 대기

SCENARIOS = [
    ("1_baseline", "완전히 벌린 상태로 손대지 말고 두세요 (기준 노이즈 측정)", False),
    ("2_air", "그리퍼 사이를 비워두세요 (허공에서 닫습니다)", True),
    ("3_hard", "두껍고 단단한 물체를 그리퍼 사이에 두세요", True),
    ("4_soft", "얇거나 무른 물체를 그리퍼 사이에 두세요", True),
    ("5_selfclose", "다시 비워두세요 (끝까지 닫아 그리퍼 자체 간섭 측정)", True),
]


# ═══════════════════════════════════════════════════════════


class Probe:
    def __init__(self):
        self.port = PortHandler(DEVICENAME)
        self.ph, _n = _make_packet_handler(self.port)
        if self.ph is None:
            sys.exit("[에러] 패킷 핸들러 생성 실패")
        print(f"[SDK] {_n}")
        self.rows = []
        self.results = {}

        if not self.port.openPort():
            sys.exit(f"[에러] 포트 {DEVICENAME} 를 열 수 없습니다")
        if not self.port.setBaudRate(BAUDRATE):
            sys.exit(f"[에러] 통신속도 {BAUDRATE} 설정 실패")

        self.open_tick, self.close_tick = GRIPPER_RANGE[GRIPPER_NAME]
        self.sign = 1 if self.close_tick > self.open_tick else -1

        self.ph.write1ByteTxRx(self.port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)
        print(
            f"[준비] {DEVICENAME} / ID {GRIPPER_ID} / {GRIPPER_NAME} "
            f"({self.open_tick} → {self.close_tick})"
        )

    # ───────────────────────────────────────────────────────
    def read_load(self):
        raw, comm, err = self.ph.read2ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_LOAD)
        if comm != 0 or err != 0:
            return None
        return raw & LOAD_MAGNITUDE_MASK

    def goto(self, tick):
        self.ph.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_POSITION, int(tick))

    def open_fully(self):
        self.goto(self.open_tick)
        time.sleep(SETTLE_DELAY)

    # ───────────────────────────────────────────────────────
    def run_static(self, tag):
        """움직이지 않고 3초간 부하만 읽는다."""
        loads = []
        t0 = time.time()
        while time.time() - t0 < 3.0:
            v = self.read_load()
            if v is not None:
                loads.append(v)
                self.rows.append([tag, round(time.time() - t0, 3), self.open_tick, v])
            time.sleep(0.05)
        return loads

    def run_closing(self, tag):
        """개방에서 조임 방향으로 스텝 이동하며 부하를 기록한다."""
        loads = []
        tick = self.open_tick
        t0 = time.time()
        aborted = False

        while (self.close_tick - tick) * self.sign > 0:
            tick += STEP_TICK * self.sign
            if (tick - self.close_tick) * self.sign > 0:
                tick = self.close_tick
            self.goto(tick)
            time.sleep(STEP_DELAY)

            v = self.read_load()
            if v is None:
                continue
            loads.append(v)
            self.rows.append([tag, round(time.time() - t0, 3), tick, v])
            print(f"    tick {tick:5d}   load {v:4d}", end="\r")

            if v >= ABORT_LOAD:
                print(f"\n    ⚠ 부하 {v} — 안전 한계 도달, 중단합니다")
                aborted = True
                break

        print()
        self.open_fully()
        return loads, aborted

    # ───────────────────────────────────────────────────────
    def run(self):
        for tag, prompt, closing in SCENARIOS:
            print("\n" + "─" * 60)
            print(f"[{tag}]  {prompt}")
            input("       준비되면 Enter (건너뛰려면 s + Enter): ").strip()

            self.open_fully()
            if closing:
                loads, aborted = self.run_closing(tag)
            else:
                loads, aborted = self.run_static(tag), False

            if not loads:
                print("       측정값 없음 — 통신을 확인하세요")
                continue

            self.results[tag] = {
                "min": min(loads),
                "max": max(loads),
                "avg": sum(loads) / len(loads),
                "n": len(loads),
                "aborted": aborted,
            }
            print(
                f"       min {min(loads)}  max {max(loads)}  "
                f"avg {sum(loads)/len(loads):.1f}  ({len(loads)}점)"
            )

    # ───────────────────────────────────────────────────────
    def save(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M")
        fname = f"gripper_load_{GRIPPER_NAME}_{stamp}.csv"
        with open(fname, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["scenario", "elapsed_s", "tick", "load"])
            w.writerows(self.rows)
        print(f"\n[저장] {fname}  ({len(self.rows)}행)")
        return fname

    def report(self):
        print("\n" + "═" * 60)
        print(f"  {GRIPPER_NAME} 부하 프로파일 요약")
        print("═" * 60)
        for tag, r in self.results.items():
            flag = "  ⚠중단" if r["aborted"] else ""
            print(
                f"  {tag:14s}  min {r['min']:4d}   max {r['max']:4d}   "
                f"avg {r['avg']:6.1f}{flag}"
            )

        air = self.results.get("2_air")
        soft = self.results.get("4_soft")
        if not (air and soft):
            print("\n  시나리오 2 또는 4 가 없어 임계값을 계산할 수 없습니다")
            return

        l_air = air["max"]  # 허공에서 나온 최대 부하 (하한)
        l_soft = soft["max"]  # 무른 물체 접촉 시 부하 (상한)

        print(f"\n  허공 최대 부하      {l_air}   ← 이보다 커야 오탐 없음")
        print(f"  무른물체 접촉 부하  {l_soft}   ← 이보다 작아야 감지됨")

        if l_soft <= l_air:
            print("\n  ⚠ 무른 물체 부하가 허공 부하보다 낮거나 같습니다.")
            print("     → 더 무른 물체로 재측정하거나, 이 그리퍼는")
            print("        접촉 감지 대상에서 제외해야 합니다.")
            return

        margin = l_soft - l_air
        recommended = int(l_air + margin * 0.4)
        print(f"\n  ▶ 권장 임계값: {recommended}   (여유폭 {margin})")
        if margin < 30:
            print("     ⚠ 여유폭이 좁습니다. 오탐/미탐이 잦을 수 있으니")
            print("        연속 프레임 조건을 3 → 5 로 올리세요.")
        print(f"\n  gripper_contact.py 에 입력:")
        print(f'      CONTACT_LOAD_THRESHOLD["{GRIPPER_NAME}"] = {recommended}')

    def shutdown(self):
        try:
            self.open_fully()
            self.ph.write1ByteTxRx(self.port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0)
        finally:
            self.port.closePort()
        print("[종료] 개방 복귀 및 토크 해제 완료")


# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    p = Probe()
    try:
        p.run()
        p.save()
        p.report()
    except KeyboardInterrupt:
        print("\n[중단] 사용자 종료")
        if p.rows:
            p.save()
    finally:
        p.shutdown()
