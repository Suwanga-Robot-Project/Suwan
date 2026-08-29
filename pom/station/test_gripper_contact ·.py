"""
test_gripper_contact.py
─────────────────────────────────────────────────────────────
하드웨어 없이 GripperContactDetector 의 판정 로직만 검증한다.
서보도 라파도 필요 없다. 오늘 밤에 돌려서 로직을 확정해둘 것.

사용법:
    python test_gripper_contact.py
"""

import gripper_contact as gc

# 테스트를 위해 임계값을 임시로 주입
gc.CONTACT_LOAD_THRESHOLD["fine"] = 300


class FakeServo:
    """부하값 시퀀스를 미리 정해두고 순서대로 뱉는 가짜 서보."""

    def __init__(self, load_sequence):
        self.seq = list(load_sequence)
        self.i = 0
        self.read_count = 0

    def read2ByteTxRx(self, port, sid, addr):
        self.read_count += 1
        v = self.seq[min(self.i, len(self.seq) - 1)]
        self.i += 1
        return v, 0, 0  # (raw, comm, err)


def make(load_sequence):
    fake = FakeServo(load_sequence)
    det = gc.GripperContactDetector("test", fake, None, 15, verbose=False)
    return det, fake


def check(name, got, want):
    ok = "PASS" if got == want else "FAIL"
    mark = "  " if ok == "PASS" else "→ "
    print(f"{mark}[{ok}] {name}:  got {got}, want {want}")
    return ok == "PASS"


# ═══════════════════════════════════════════════════════════
results = []
print("=" * 60)

# ── 1. 허공에서 닫기: 부하가 낮으면 그대로 통과 ──────────
det, fake = make([50, 60, 55, 58])
out = [det.apply(2400 + i * 10, 2390 + i * 10, "fine") for i in range(4)]
results.append(check("허공 닫기 — 목표 통과", out, [2400, 2410, 2420, 2430]))
results.append(check("허공 닫기 — 래치 안 걸림", det.latched, False))

# ── 2. 부하가 임계를 넘으면 연속 3프레임 후 정지 ─────────
det, fake = make([50, 350, 360, 370, 380])
out = []
prev = 2400
for i in range(5):
    tgt = prev + 10
    got = det.apply(tgt, prev, "fine")
    out.append(got)
    prev = got
# 1프레임: 50 → 통과(2410), 2·3·4프레임에서 3연속 초과 → 4번째에 래치
results.append(check("접촉 감지 — 래치 걸림", det.latched, True))
results.append(check("접촉 감지 — 정지 후 tick 고정", out[-1] == out[-2], True))

# ── 3. 래치 상태에서 더 조이려 해도 막힌다 ───────────────
locked_tick = det.contact_tick
got = det.apply(locked_tick + 200, locked_tick, "fine")
results.append(check("래치 중 조임 명령 차단", got, locked_tick))

# ── 4. 여는 방향 명령은 즉시 통과하고 래치가 풀린다 ──────
open_cmd = locked_tick - (gc.RELEASE_MARGIN + 5)
got = det.apply(open_cmd, locked_tick, "fine")
results.append(check("여는 명령 통과", got, open_cmd))
results.append(check("여는 명령으로 래치 해제", det.latched, False))

# ── 5. 임계값 미실측이면 아무것도 안 한다 ────────────────
det, fake = make([900, 900, 900, 900])
out = [det.apply(2400 + i * 10, 2390 + i * 10, "default") for i in range(4)]
results.append(check("미실측 그리퍼 — 통과", out, [2400, 2410, 2420, 2430]))
results.append(check("미실측 그리퍼 — 폴링 안 함", fake.read_count, 0))

# ── 6. 니퍼는 제외 대상이라 통과 ─────────────────────────
gc.CONTACT_LOAD_THRESHOLD["nipper"] = 300
det, fake = make([900, 900, 900])
out = [det.apply(2400 + i * 10, 2390 + i * 10, "nipper") for i in range(3)]
results.append(check("니퍼 제외 — 통과", out, [2400, 2410, 2420]))
results.append(check("니퍼 제외 — 폴링 안 함", fake.read_count, 0))

# ── 7. 여는 중에는 폴링하지 않는다 (통신 부하 0) ─────────
det, fake = make([900, 900, 900])
for i in range(3):
    det.apply(2400 - i * 10, 2410 - i * 10, "fine")
results.append(check("여는 중 — 폴링 안 함", fake.read_count, 0))

# ── 8. 정지 중에도 폴링하지 않는다 ───────────────────────
det, fake = make([900, 900, 900])
for _ in range(3):
    det.apply(2400, 2400, "fine")
results.append(check("정지 중 — 폴링 안 함", fake.read_count, 0))


# ── 9. 통신 실패가 누적되면 기능이 자동 정지한다 ─────────
class FailingServo:
    def __init__(self):
        self.n = 0

    def read2ByteTxRx(self, port, sid, addr):
        self.n += 1
        return 0, -1, 0  # comm 에러


det = gc.GripperContactDetector("test", FailingServo(), None, 15, verbose=False)
prev = 2400
for i in range(gc.MAX_READ_FAILURES + 2):
    got = det.apply(prev + 10, prev, "fine")
    prev = got
results.append(check("통신 실패 누적 — 기능 정지", det.enabled, False))
results.append(check("통신 실패 중에도 그리퍼는 움직임", prev > 2400, True))

# ── 10. reset 이 래치를 지운다 ───────────────────────────
det, fake = make([400] * 6)
prev = 2400
for _ in range(5):
    prev = det.apply(prev + 10, prev, "fine")
was = det.latched
det.reset("테스트")
results.append(check("reset 전 래치 상태", was, True))
results.append(check("reset 후 래치 해제", det.latched, False))

# ═══════════════════════════════════════════════════════════
print("=" * 60)
passed = sum(1 for r in results if r)
print(f"  {passed} / {len(results)} 통과")
if passed != len(results):
    print("  ⚠ 실패 항목을 먼저 고친 뒤 실기로 넘어갈 것")
print("=" * 60)
