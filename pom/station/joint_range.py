"""
joint_range_probe.py
─────────────────────────────────────────────────────────────
팔 관절(1~6번)의 기구 가동범위를 서보 구동으로 자동 측정한다.

gripper_dir_check.py 의 creep() 방식을 그대로 가져와 12축용으로 확장한 것.
손으로 관절을 못 움직이는 경우(감속비가 커서 역구동이 안 됨)를 전제로,
서보가 직접 조금씩 밀면서 "더 이상 안 가는 지점"을 찾는다.

측정 결과는 station_positions.py 의 JOINT_LIMITS_LEFT / JOINT_LIMITS_RIGHT 에
붙여넣을 수 있는 형태로 출력된다.

안전장치
    · 한 번에 STEP_TICK(10)씩만 이동
    · 시작 시 정지 상태의 부하를 재고, 거기서 LOAD_MARGIN 만큼 올라가면 중단
      (중력 때문에 어깨 관절은 가만히 있어도 부하가 높다 — 고정 임계값은 못 씀)
    · 목표에 STALL_MARGIN 이상 못 미치는 상태가 STALL_FRAMES 연속이면 끝단 판정
    · 한 방향 총 이동량이 MAX_TRAVEL 을 넘으면 중단
    · 끝단에서 한 스텝 물러나 힘을 빼고, 각 관절 측정 후 시작 위치로 복귀
    · Ctrl+C 로 중단해도 시작 위치 복귀 후 종료

⚠ 실행 전 확인
    1. 팔 주변에 사람/장애물이 없을 것
    2. 그리퍼는 이 스크립트 대상이 아님 → gripper_dir_check.py 사용
    3. COM 포트를 쓰는 프로그램(N5.py 등)을 전부 종료할 것
    4. 비상시 바로 끌 수 있게 전원 스위치 근처에 있을 것

사용법
    python joint_range_probe.py               # 양팔 1~6번 전부
    python joint_range_probe.py left          # 왼팔만
    python joint_range_probe.py right 3       # 오른팔 3번만 다시
"""

import inspect
import sys
import time

from scservo_sdk import *  # noqa: F401,F403
import scservo_sdk as _sdk

# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전 확인
# ═══════════════════════════════════════════════════════════

PORT_LEFT = "COM12"
PORT_RIGHT = "COM14"

MOTORS_LEFT = [1, 2, 3, 4, 5, 6]  # 7번(그리퍼) 제외
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14]  # 15번(그리퍼) 제외

BAUDRATE = 1000000
PROTOCOL_END = 0

# ═══════════════════════════════════════════════════════════
#  안전 파라미터
# ═══════════════════════════════════════════════════════════

STEP_TICK = 10  # 한 스텝 이동량
STEP_DELAY = 0.08  # 스텝 간 대기 (서보가 따라올 시간)

# 중력 부하 대응: 시작 시 정지 부하를 재고 거기서 이만큼 올라가면 중단.
# 어깨(2·3번)는 가만히 있어도 부하가 높게 나올 수 있어서
# gripper_dir_check.py 처럼 고정값(450)을 쓰면 시작하자마자 걸린다.
LOAD_MARGIN = 250
LOAD_ABSOLUTE_MAX = 800  # 기준부하가 아무리 높아도 여기는 절대 넘기지 않음

STALL_MARGIN = 25  # 목표 대비 이만큼 못 가면 정체로 간주
STALL_FRAMES = 3  # 연속 몇 프레임 정체하면 끝단으로 판정
MAX_TRAVEL = 2600  # 한 방향 최대 이동량 (폭주 방지)

SAFETY_MARGIN = 60  # 실측 끝단에서 안쪽으로 좁힐 tick 수
MIN_USEFUL_SPAN = SAFETY_MARGIN * 3  # 가동폭이 이보다 좁으면 측정 실패로 봄

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
POS_MIN, POS_MAX = 0, 4095


# ═══════════════════════════════════════════════════════════
#  SDK 호환 (gripper_dir_check.py 와 동일)
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
        if hasattr(ph, "read2ByteTxRx") and hasattr(ph, "write1ByteTxRx"):
            return ph, name
    return None, None


def swap16(v):
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


# ═══════════════════════════════════════════════════════════
#  관절 하나를 다루는 헬퍼
# ═══════════════════════════════════════════════════════════


class Joint:
    def __init__(self, ph, port, motor_id, label):
        self.ph = ph
        self.port = port
        self.mid = motor_id
        self.label = label

    def pos(self):
        v, c, e = self.ph.read2ByteTxRx(self.port, self.mid, ADDR_PRESENT_POSITION)
        if c != 0 or e != 0:
            return None
        if POS_MIN <= v <= POS_MAX:
            return v
        sw = swap16(v)
        return sw if POS_MIN <= sw <= POS_MAX else None

    def load(self):
        v, c, e = self.ph.read2ByteTxRx(self.port, self.mid, ADDR_PRESENT_LOAD)
        return (v & 0x03FF) if (c == 0 and e == 0) else 0

    def torque(self, on):
        self.ph.write1ByteTxRx(self.port, self.mid, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def goto(self, tick):
        t = max(POS_MIN, min(POS_MAX, int(tick)))
        self.ph.write2ByteTxRx(self.port, self.mid, ADDR_GOAL_POSITION, t)

    def baseline_load(self, samples=10):
        """정지 상태 부하의 평균. 중력 부하를 반영한 기준선."""
        vals = []
        for _ in range(samples):
            vals.append(self.load())
            time.sleep(0.05)
        return sum(vals) / len(vals) if vals else 0

    def creep(self, direction, abort_load, tag):
        """
        direction(+1/-1)으로 천천히 밀면서 끝단을 찾는다.
        반환: (끝단 tick 또는 None, 중단 사유)
        """
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
                f"    {tag}  목표 {tick:4d}  실제 {actual:4d}  "
                f"부하 {ld:4d}/{abort_load:4d}  이동 {traveled:4d}",
                end="\r",
            )

            if ld >= abort_load:
                reason = f"부하 한계 {ld}"
                break

            if abs(actual - tick) > STALL_MARGIN:
                stall += 1
                if stall >= STALL_FRAMES:
                    reason = "기계적 끝단"
                    break
            else:
                stall = 0

        print(" " * 78, end="\r")

        # 끝단에 힘주고 있지 않도록 한 스텝 물러남
        final = self.pos()
        if final is not None:
            self.goto(final - STEP_TICK * direction)
            time.sleep(0.3)
            final = self.pos()
        return final, reason

    def return_to(self, tick, settle=1.2):
        if tick is None:
            return
        self.goto(tick)
        time.sleep(settle)


# ═══════════════════════════════════════════════════════════
#  측정
# ═══════════════════════════════════════════════════════════


def probe_joint(joint):
    print()
    print("─" * 62)
    print(f"  {joint.label}   (서보 ID {joint.mid})")
    print("─" * 62)

    start = joint.pos()
    if start is None:
        print("  [실패] 위치를 못 읽었습니다. 서보 ID / 포트를 확인하세요.")
        return None

    print(f"  현재 위치 {start}")
    print("  정지 상태 부하 측정 중...", end="", flush=True)
    base = joint.baseline_load()
    abort_load = int(min(LOAD_ABSOLUTE_MAX, base + LOAD_MARGIN))
    print(f" 기준 {base:.0f}  →  중단 임계값 {abort_load}")

    ans = input("  이 관절을 측정할까요? (Enter=진행 / s=건너뛰기) > ").strip().lower()
    if ans == "s":
        print("  건너뜀")
        return None

    lo = hi = None
    r1 = r2 = "미측정"
    try:
        print("\n  [1/2] tick 증가 방향으로 끝까지...")
        hi, r1 = joint.creep(+1, abort_load, "증가")
        print(f"        끝단 {hi}   ({r1})")

        print("  시작 위치로 복귀 중...")
        joint.return_to(start)

        print("\n  [2/2] tick 감소 방향으로 끝까지...")
        lo, r2 = joint.creep(-1, abort_load, "감소")
        print(f"        끝단 {lo}   ({r2})")
    finally:
        print("  시작 위치로 복귀 중...")
        joint.return_to(start)

    if lo is None or hi is None:
        print("  [실패] 한쪽 끝단을 못 찾았습니다.")
        return None

    if lo > hi:
        lo, hi = hi, lo

    span = hi - lo
    print(f"\n  ▶ 가동범위 {lo} ~ {hi}   (폭 {span} tick)")
    if span < MIN_USEFUL_SPAN:
        print(f"  ⚠ 가동폭이 {MIN_USEFUL_SPAN} 미만입니다.")
        print("     LOAD_MARGIN 을 올리거나, 기구적 문제가 없는지 확인하세요.")

    return (lo, hi, span, r1, r2)


def probe_arm(arm_side, only_index=None):
    port_name = PORT_LEFT if arm_side == "left" else PORT_RIGHT
    motors = MOTORS_LEFT if arm_side == "left" else MOTORS_RIGHT
    arm_label = "왼팔" if arm_side == "left" else "오른팔"

    port = PortHandler(port_name)
    ph, sdk_name = _make_packet_handler(port)
    if ph is None:
        print(f"[에러] {arm_label} 패킷 핸들러 생성 실패")
        return None

    try:
        port.openPort()
        port.setBaudRate(BAUDRATE)
    except Exception as e:
        print(f"\n[에러] {port_name} 열기 실패: {e}")
        print("       N5.py / 시리얼 모니터를 모두 종료했는지 확인하세요")
        return None

    print()
    print("═" * 62)
    print(f"  {arm_label}  ({port_name})   SDK 방식: {sdk_name}")
    print("═" * 62)

    targets = range(len(motors)) if only_index is None else [only_index]
    results = {}

    joints = [
        Joint(ph, port, motors[i], f"{arm_label} {i+1}번") for i in range(len(motors))
    ]

    try:
        for i in targets:
            if i >= len(motors):
                continue
            joints[i].torque(True)
            time.sleep(0.2)
            r = probe_joint(joints[i])
            if r is not None:
                results[i] = r
    except KeyboardInterrupt:
        print("\n\n  [중단] 사용자 종료")
    finally:
        port.closePort()
        print(f"\n  {arm_label} 포트 닫음 (토크는 유지 — 팔이 떨어지지 않게)")

    return results


def print_report(arm_side, results):
    arm_label = "왼팔" if arm_side == "left" else "오른팔"
    var_name = "JOINT_LIMITS_LEFT" if arm_side == "left" else "JOINT_LIMITS_RIGHT"

    print()
    print("═" * 62)
    print(f"  {arm_label} 결과 — station_positions.py 에 붙여넣기")
    print("═" * 62)
    print()
    print(f"{var_name} = [")

    for i in range(6):
        if i not in results:
            print(f"    None,  # {i+1}번 — 이번에 측정 안 함")
            continue
        lo, hi, span, r1, r2 = results[i]
        if span < MIN_USEFUL_SPAN:
            print(f"    None,  # {i+1}번 — 가동폭 {span}뿐, 재측정 필요")
            continue
        print(
            f"    ({lo + SAFETY_MARGIN}, {hi - SAFETY_MARGIN}),"
            f"  # {i+1}번 — 실측 {lo}~{hi}, 폭 {span}"
        )

    print("    None,  # 7번 그리퍼 — gripper_dir_check.py 로 별도 관리")
    print("]")
    print()
    print(f"  (실측 끝단에서 양쪽 {SAFETY_MARGIN}tick 씩 안쪽으로 좁힌 값입니다)")


def main():
    args = [a.lower() for a in sys.argv[1:]]

    arm_filter = None
    index_filter = None
    for a in args:
        if a in ("left", "right"):
            arm_filter = a
        elif a.isdigit() and 1 <= int(a) <= 6:
            index_filter = int(a) - 1

    sides = [arm_filter] if arm_filter else ["left", "right"]

    print()
    print("═" * 62)
    print("  팔 관절 가동범위 자동 측정")
    print("═" * 62)
    only = f" / {index_filter+1}번만" if index_filter is not None else ""
    print(f"  대상: {', '.join(sides)}{only}")
    print(f"  한 스텝 {STEP_TICK}tick, 안전여유 {SAFETY_MARGIN}tick")
    print()
    print("  ⚠ 팔 주변에 사람이나 장애물이 없는지 확인하세요.")
    print("     서보가 직접 천천히 움직여서 양쪽 끝을 찾습니다.")
    print("     관절마다 진행 여부를 다시 물어봅니다.")
    input("\n  준비되면 Enter (Ctrl+C로 중단) > ")

    for side in sides:
        results = probe_arm(side, index_filter)
        if results:
            print_report(side, results)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n중단됨. 서보 상태를 확인하세요.")
