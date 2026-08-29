"""
comm_reliability_test.py
─────────────────────────────────────────────────────────────
서보 버스 통신 신뢰성 진단 — "특정 모터 고장" 가설을 데이터로 기각한다

증상
    그리퍼 스테이션 위치 이동 시, 7개 관절 중 특정 모터 하나가 목표에
    도달하지 못하고 일정 오차로 고정된다.

무엇을 가르는가
    ① 그 모터의 하드웨어 문제인가
    ② 7개 모터에 연속으로 write/read 할 때 생기는 버스 타이밍 문제인가

    핵심은 '움직임을 배제하는 것'이다. 팔이 크게 움직이면 백래시·관성·
    중력 때문에 "명령대로 안 갔다"가 통신 실패인지 기구 문제인지 구분되지
    않는다. 그래서 체인 단계에서는 같은 값을 반복해서 쓰고 읽어,
    남는 것이 패킷이 오갔느냐 아니냐뿐이 되게 만든다.

6단계 구성
    1) scan       버스 스캔 — 어떤 ID가 응답하는가
    2) status     의심 모터의 Load / Voltage / Temperature
    3) manual     단일 명령 검증 — 목표 하나를 주고 도달 오차 확인
    4) solo       단독 구동 — 그 모터 혼자만 여러 번 왕복
    5) chain      체인 read — 7개 연속, 지연 유무 대조   ★ 핵심
    6) chainwrite 체인 write/read — 동일 값 반복, 도달 오차 기록

CSV 출력 형식
    scan,        id,     ok,       model
    status,      id,     load,     voltage,  temp,   moving
    manual,      target, actual,   error
    solo,        target, actual,   error
    chain,       delay,  attempts, fail,     error
    chainwrite,  index,  target,   actual,   error

⚠ 실행 전
    · N5.py / 시리얼 모니터 등 COM 포트를 쓰는 프로그램을 전부 종료할 것
    · solo 단계에서 관절이 실제로 움직이므로 주변을 비워둘 것

사용법
    python comm_reliability_test.py            # 오른팔 (기본)
    python comm_reliability_test.py left       # 왼팔
"""

import csv
import inspect
import sys
import time
from datetime import datetime

from scservo_sdk import *  # noqa: F401,F403
import scservo_sdk as _sdk

# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전 확인
# ═══════════════════════════════════════════════════════════

PORT_LEFT = "COM12"
PORT_RIGHT = "COM14"

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

# 집중 진단할 모터 (증상이 나타난 모터)
SUSPECT_LEFT = 1  # 왼팔 1번
SUSPECT_RIGHT = 11  # 오른팔 3번

BAUDRATE = 1000000
PROTOCOL_END = 0

# ═══════════════════════════════════════════════════════════
#  측정 파라미터
# ═══════════════════════════════════════════════════════════

CHAIN_ROUNDS = 30  # 체인 read 회전 수 → 30 × 7모터 = 210회 시도
CHAIN_DELAYS = (0.0, 0.008)  # 대조할 모터 간 지연 (초)

CHAINWRITE_REPEATS = 5  # 체인 write/read 반복 횟수
CHAINWRITE_STEP = 100  # 두 목표값의 간격 (tick)

SOLO_REPEATS = 3  # 단독 구동 반복 횟수
SOLO_STEP = 100  # 단독 구동 이동량 (tick)
SETTLE = 0.5  # 이동 후 안정화 대기 (초)

POS_MIN, POS_MAX = 0, 4095

ADDR_MODEL_NUMBER = 3
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66


# ═══════════════════════════════════════════════════════════
#  SDK 호환 (설치된 버전에 따라 생성 방식이 다름)
# ═══════════════════════════════════════════════════════════


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
        if hasattr(ph, "read2ByteTxRx") and hasattr(ph, "write2ByteTxRx"):
            return ph, name
    return None, None


def swap16(v):
    """상하위 바이트를 뒤집는다 (엔디안 불일치 보정)."""
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def hr(c="─", n=64):
    print(c * n)


def step(t):
    print()
    hr()
    print(f"  {t}")
    hr()


# ═══════════════════════════════════════════════════════════
#  버스
# ═══════════════════════════════════════════════════════════


class Bus:
    def __init__(self, name, port_name, motor_ids):
        self.name = name
        self.ids = motor_ids
        self.ok = False

        self.port = PortHandler(port_name)
        self.ph, self.sdk_name = _make_packet_handler(self.port)
        if self.ph is None:
            print(f"  [{name}] 패킷 핸들러 생성 실패")
            return

        try:
            if not self.port.openPort():
                print(f"  [{name}] {port_name} 열기 실패")
                print("        N5.py / 시리얼 모니터를 모두 종료했는지 확인하세요")
                return
            if not self.port.setBaudRate(BAUDRATE):
                print(f"  [{name}] 통신속도 설정 실패")
                return
        except Exception as e:
            print(f"  [{name}] {port_name} 오류: {e}")
            return

        self.ok = True
        print(f"  [{name}] {port_name} 연결   SDK: {self.sdk_name}")

    # ───────────────────────────────────────────────────────
    def _read2(self, mid, addr, lo=None, hi=None):
        try:
            v, comm, err = self.ph.read2ByteTxRx(self.port, mid, addr)
        except Exception:
            return None
        if comm != 0 or err != 0:
            return None
        if lo is None:
            return v
        if lo <= v <= hi:
            return v
        sw = swap16(v)
        return sw if lo <= sw <= hi else None

    def _read1(self, mid, addr):
        try:
            v, comm, err = self.ph.read1ByteTxRx(self.port, mid, addr)
        except Exception:
            return None
        return None if (comm != 0 or err != 0) else v

    def pos(self, mid):
        return self._read2(mid, ADDR_PRESENT_POSITION, POS_MIN, POS_MAX)

    def model(self, mid):
        return self._read2(mid, ADDR_MODEL_NUMBER)

    def load(self, mid):
        v = self._read2(mid, ADDR_PRESENT_LOAD)
        return None if v is None else (v & 0x03FF)

    def voltage(self, mid):
        return self._read1(mid, ADDR_PRESENT_VOLTAGE)

    def temperature(self, mid):
        return self._read1(mid, ADDR_PRESENT_TEMPERATURE)

    def moving(self, mid):
        return self._read1(mid, ADDR_MOVING)

    def goto(self, mid, tick):
        t = max(POS_MIN, min(POS_MAX, int(tick)))
        try:
            comm, err = self.ph.write2ByteTxRx(self.port, mid, ADDR_GOAL_POSITION, t)
        except Exception:
            return False
        return comm == 0 and err == 0

    def torque(self, mid, on):
        try:
            self.ph.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 1 if on else 0)
        except Exception:
            pass

    def close(self):
        if self.ok:
            try:
                self.port.closePort()
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════
#  1. 버스 스캔
# ═══════════════════════════════════════════════════════════


def phase_scan(bus, rows):
    step("1. 버스 스캔 — 어떤 모터가 응답하는가")

    alive = []
    for mid in bus.ids:
        m = bus.model(mid)
        ok = 1 if m is not None else 0
        idx = bus.ids.index(mid) + 1
        print(
            f"    {bus.name} {idx}번 (ID {mid:2d})   "
            f"{'응답' if ok else '무응답'}   model={m}"
        )
        rows.append(["scan", mid, ok, m if m is not None else ""])
        if ok:
            alive.append(mid)

    print()
    if len(alive) == len(bus.ids):
        print(f"    ✓ {len(alive)}개 전부 응답 — 버스·배선 정상")
    else:
        missing = [m for m in bus.ids if m not in alive]
        print(f"    ⚠ 무응답 {len(missing)}개: {missing}")
        print("      배선을 먼저 확인하세요. 이 상태로는 아래 단계가 무의미합니다.")
    return alive


# ═══════════════════════════════════════════════════════════
#  2. 상태 레지스터
# ═══════════════════════════════════════════════════════════


def phase_status(bus, suspect, rows):
    step("2. 상태 레지스터 — 기구적 스톨(걸림) 가설 검증")
    print("  Load가 높고 온도가 오르면 물리적으로 막힌 것이고,")
    print("  Load가 낮은데 위치가 안 바뀌면 통신·제어 계층 문제입니다.\n")

    bus.torque(suspect, True)
    time.sleep(0.2)

    load = bus.load(suspect)
    volt = bus.voltage(suspect)
    temp = bus.temperature(suspect)
    mov = bus.moving(suspect)

    idx = bus.ids.index(suspect) + 1 if suspect in bus.ids else "?"
    print(f"    {bus.name} {idx}번 (ID {suspect})")
    print(f"      Load        {load}")
    if volt is not None:
        print(f"      Voltage     {volt}  ({volt / 10:.1f}V)")
    else:
        print("      Voltage     읽기 실패")
    print(f"      Temperature {temp}°C")
    print(f"      Moving      {mov}")

    rows.append(
        [
            "status",
            suspect,
            load if load is not None else "",
            volt if volt is not None else "",
            temp if temp is not None else "",
            mov if mov is not None else "",
        ]
    )

    print()
    if load is not None and load < 200 and temp is not None and temp < 45:
        print("    → Load 낮고 온도 상승 없음. 기구적 스톨 가설 기각")
    elif load is not None and load >= 200:
        print("    ⚠ Load가 높습니다 — 기구적으로 막혀 있을 수 있습니다")


# ═══════════════════════════════════════════════════════════
#  3. 단일 명령 검증
# ═══════════════════════════════════════════════════════════


def phase_manual(bus, suspect, rows):
    step("3. 단일 명령 검증 — 목표 하나를 주고 도달 오차 확인")

    cur = bus.pos(suspect)
    if cur is None:
        print("    위치 읽기 실패 — 건너뜁니다")
        return

    print(f"    현재 위치 {cur}")
    raw = input(f"    목표 tick 입력 (Enter = 현재값 {cur} 유지): ").strip()
    target = int(raw) if raw.isdigit() else cur

    bus.torque(suspect, True)
    bus.goto(suspect, target)
    time.sleep(SETTLE)
    actual = bus.pos(suspect)

    if actual is None:
        print("    도달 위치 읽기 실패")
        return

    err = actual - target
    print(f"    목표 {target}  →  실제 {actual}   오차 {err:+d}")
    rows.append(["manual", target, actual, err])


# ═══════════════════════════════════════════════════════════
#  4. 단독 구동
# ═══════════════════════════════════════════════════════════


def phase_solo(bus, suspect, rows):
    step("4. 단독 구동 — 그 모터 혼자만 움직일 때는 정상인가")
    print("  다른 모터에 명령을 보내지 않고 이 모터만 왕복시킵니다.")
    print("  여기서 정상이면 모터 자체는 멀쩡하다는 뜻입니다.\n")
    print("  ⚠ 이 단계에서는 관절이 실제로 움직입니다. 주변을 비워두세요.")
    input("    준비되면 Enter > ")

    base = bus.pos(suspect)
    if base is None:
        print("    위치 읽기 실패 — 건너뜁니다")
        return

    bus.torque(suspect, True)
    errors = []

    for i in range(SOLO_REPEATS):
        target = base + (SOLO_STEP if i % 2 == 0 else 0)
        bus.goto(suspect, target)
        time.sleep(SETTLE)
        actual = bus.pos(suspect)
        if actual is None:
            print(f"    {i + 1}회차 — 위치 읽기 실패")
            continue
        err = actual - target
        errors.append(abs(err))
        print(f"    {i + 1}회차   목표 {target}  →  실제 {actual}   오차 {err:+d}")
        rows.append(["solo", target, actual, err])

    bus.goto(suspect, base)
    time.sleep(SETTLE)

    print()
    if errors and max(errors) <= 10:
        print(f"    → 단독 구동 오차 최대 {max(errors)}. 모터 자체는 정상")
    elif errors:
        print(f"    ⚠ 단독으로도 오차 {max(errors)} — 모터·기구 문제 의심")


# ═══════════════════════════════════════════════════════════
#  5. 체인 read  ★ 핵심
# ═══════════════════════════════════════════════════════════


def phase_chain(bus, alive, rows):
    step("5. 체인 read — 7개 연속 읽기, 지연 유무 대조  ★")
    print("  움직이지 않는 상태에서 7개 모터를 연속으로 읽습니다.")
    print("  순수 통신 성공률만 남습니다.\n")

    results = {}

    for delay in CHAIN_DELAYS:
        label = "지연 없음" if delay == 0 else f"지연 {delay * 1000:.0f}ms"
        fails = {mid: 0 for mid in alive}
        errors = 0

        t0 = time.time()
        for _ in range(CHAIN_ROUNDS):
            for mid in alive:
                p = bus.pos(mid)
                if p is None:
                    fails[mid] += 1
                if delay:
                    time.sleep(delay)
        elapsed = time.time() - t0

        attempts = CHAIN_ROUNDS * len(alive)
        bad = sum(fails.values())
        rate = 100.0 * (attempts - bad) / attempts if attempts else 0.0
        cycle_ms = elapsed / CHAIN_ROUNDS * 1000

        print(f"    [{label}]")
        print(f"      소요 {elapsed:.2f}초   1회전 {cycle_ms:.1f} ms")
        print(f"      성공률 {rate:.2f}%   ({attempts - bad}/{attempts})")
        if bad:
            print("      실패 분포:")
            for mid, n in sorted(fails.items(), key=lambda x: -x[1]):
                if n:
                    idx = bus.ids.index(mid) + 1
                    print(f"        {bus.name} {idx}번(ID {mid}): {n}회")
        else:
            print("      실패 없음")
        print()

        rows.append(["chain", delay, attempts, bad, errors])
        results[delay] = (rate, cycle_ms)

    # ── 판정 ────────────────────────────────────────────
    if len(results) == 2:
        r0, c0 = results[CHAIN_DELAYS[0]]
        r1, c1 = results[CHAIN_DELAYS[1]]
        slower = c1 / c0 if c0 else 0

        hr()
        print("    판정")
        print(f"      지연 없음  성공률 {r0:.2f}%   1회전 {c0:.1f} ms")
        print(f"      지연 8ms   성공률 {r1:.2f}%   1회전 {c1:.1f} ms")
        print(f"      → {slower:.1f}배 느려짐")
        print()

        if r0 >= 99.9 and r1 >= 99.9:
            print("      두 조건 모두 실패가 없습니다.")
            print(f"      → INTER_MOTOR_DELAY 는 {slower:.0f}배 느려지는 대가에")
            print("         상응하는 이득이 없습니다. 완화책일 뿐 해법이 아닙니다.")
        elif r1 > r0 + 0.5:
            print("      지연을 넣었을 때만 성공률이 올랐습니다.")
            print("      → 버스 타이밍·경합 문제 확정")
        else:
            print("      지연과 무관하게 실패가 발생합니다.")
            print("      → 버스/전원/케이블 또는 개별 모터 문제")

    return results


# ═══════════════════════════════════════════════════════════
#  6. 체인 write/read
# ═══════════════════════════════════════════════════════════


def phase_chainwrite(bus, suspect, alive, rows):
    step("6. 체인 write/read — 동일 값 반복, 도달 오차 기록")
    print("  다른 모터를 계속 읽는 부하 상태에서, 의심 모터에만")
    print("  같은 두 값을 번갈아 써 넣고 실제 도달값을 기록합니다.")
    print("  큰 이동이 없으므로 백래시·관성 영향이 최소화됩니다.\n")

    base = bus.pos(suspect)
    if base is None:
        print("    위치 읽기 실패 — 건너뜁니다")
        return

    bus.torque(suspect, True)
    a, b = base, base + CHAINWRITE_STEP

    for i in range(CHAINWRITE_REPEATS):
        target = a if i % 2 == 0 else b

        # 버스에 부하를 주기 위해 다른 모터도 함께 읽는다
        for mid in alive:
            if mid != suspect:
                bus.pos(mid)

        bus.goto(suspect, target)
        time.sleep(SETTLE)
        actual = bus.pos(suspect)

        if actual is None:
            print(f"    {i}회차 — 위치 읽기 실패")
            continue

        err = actual - target
        print(f"    {i}회차   목표 {target}  →  실제 {actual}   오차 {err:+d}")
        rows.append(["chainwrite", i, target, actual, err])

    bus.goto(suspect, base)
    time.sleep(SETTLE)


# ═══════════════════════════════════════════════════════════


def main():
    args = [a.lower() for a in sys.argv[1:]]
    side = "left" if "left" in args else "right"

    if side == "left":
        name, port, ids, suspect = "왼팔", PORT_LEFT, MOTORS_LEFT, SUSPECT_LEFT
    else:
        name, port, ids, suspect = "오른팔", PORT_RIGHT, MOTORS_RIGHT, SUSPECT_RIGHT

    print()
    hr("═")
    print("  서보 버스 통신 신뢰성 진단")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    hr("═")
    idx = ids.index(suspect) + 1 if suspect in ids else "?"
    print(f"  대상: {name}   집중 진단: {idx}번 (ID {suspect})")
    print(
        f"  체인 {CHAIN_ROUNDS}회전 × {len(ids)}모터 = "
        f"{CHAIN_ROUNDS * len(ids)}회 시도"
    )
    print()
    print("  ⚠ COM 포트를 쓰는 다른 프로그램(N5.py 등)을 모두 종료하세요.")
    input("\n  준비되면 Enter (Ctrl+C로 중단) > ")

    print()
    bus = Bus(name, port, ids)
    if not bus.ok:
        return

    rows = []
    try:
        alive = phase_scan(bus, rows)
        if not alive:
            print("\n  응답하는 모터가 없습니다. 배선을 확인하세요.")
            return

        if suspect not in alive:
            print(f"\n  ⚠ 집중 진단 대상(ID {suspect})이 무응답입니다.")
            suspect = alive[0]
            print(f"     대신 ID {suspect} 로 진행합니다.")

        phase_status(bus, suspect, rows)
        phase_manual(bus, suspect, rows)
        phase_solo(bus, suspect, rows)
        phase_chain(bus, alive, rows)
        phase_chainwrite(bus, suspect, alive, rows)

    except KeyboardInterrupt:
        print("\n\n  [중단] 사용자 종료")
    finally:
        if rows:
            fn = f"comm_reliability_{side}_{datetime.now():%Y%m%d_%H%M}.csv"
            with open(fn, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerows(rows)
            print(f"\n  [저장] {fn}   ({len(rows)}행)")
        bus.close()
        print("  포트 닫음. 완료.")


if __name__ == "__main__":
    main()
