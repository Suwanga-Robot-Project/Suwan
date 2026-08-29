"""
수완 로봇 - 조그 제어 로직 자체 테스트

로봇도 STM32도 라즈베리파이도 필요 없다. 노트북에서 그냥 실행하면 된다.
jog_control.py 의 로직이 의도대로 동작하는지 9가지 시나리오로 검증한다.

실행:
    python jog_selftest.py
"""

import jog_control as jc
from jog_control import JogController

DT = 0.021  # 루프 주기 (실측 20.95ms)
NEUTRAL = jc.JOY_NEUTRAL
PUSH_MAX = 4000  # 조이스틱을 끝까지 민 상태
PUSH_MIN = 60  # 반대 방향 끝

_passed = 0
_failed = 0


def check(label, condition, detail=""):
    global _passed, _failed
    if condition:
        _passed += 1
        print("  [PASS] %s" % label)
    else:
        _failed += 1
        print("  [FAIL] %s   %s" % (label, detail))


def run(ctrl, frames, gate=0, key7=0, jx=NEUTRAL, jy=NEUTRAL, closed=False):
    """지정한 입력으로 frames 만큼 루프를 돌린다."""
    out = None
    for _ in range(frames):
        out = ctrl.update(
            gate_toggle=gate,
            key7=key7,
            joy_x=jx,
            joy_y=jy,
            gripper_closed=closed,
            dt=DT,
        )
    return out


def press_key(ctrl, gate=0, closed=False, jx=NEUTRAL, jy=NEUTRAL):
    """키캡을 눌렀다 떼는 동작 (누름 2프레임 + 뗌 2프레임)."""
    run(ctrl, 2, gate=gate, key7=1, jx=jx, jy=jy, closed=closed)
    run(ctrl, 2, gate=gate, key7=0, jx=jx, jy=jy, closed=closed)


# ============================================================
print("\n[1] 평상시 - 토글 0, 키 0, 조이스틱 중립")
# ============================================================
c = JogController("L")
off = run(c, 50)
check("오프셋이 0이다", off[jc.IDX_UD] == 0 and off[jc.IDX_LR] == 0, off)
check("조그가 비활성이다", c.active is False)


# ============================================================
print("\n[2] 조이스틱만 밀어도 조그가 안 켜진다 (키 안 누름)")
# ============================================================
c = JogController("L")
off = run(c, 100, jy=PUSH_MAX)
check("오프셋이 0이다", off[jc.IDX_UD] == 0, off)


# ============================================================
print("\n[3] 키캡 7번을 누르면 조그가 켜진다")
# ============================================================
c = JogController("L")
press_key(c)
check("조그 활성", c.active is True)
check("아직 오프셋은 0", c.is_offset_zero())


# ============================================================
print("\n[4] 조그 ON 상태에서 조이스틱 상하 밀기")
# ============================================================
off = run(c, 100, jy=PUSH_MAX)  # 약 2.1초
check("상하 오프셋이 커졌다", off[jc.IDX_UD] > 5, off[jc.IDX_UD])
check("좌우 오프셋은 그대로 0", off[jc.IDX_LR] == 0, off[jc.IDX_LR])

off = run(c, 400, jy=PUSH_MAX)  # 계속 밀기
check("클램프에 걸려 멈춘다", off[jc.IDX_UD] == int(jc.CLAMP_UD), off[jc.IDX_UD])


# ============================================================
print("\n[5] 조이스틱을 놓으면 오프셋이 유지된다")
# ============================================================
before = off[jc.IDX_UD]
off = run(c, 100)  # 중립으로 2초
check("오프셋 유지", off[jc.IDX_UD] == before, "%s -> %s" % (before, off[jc.IDX_UD]))


# ============================================================
print("\n[6] 물체를 쥔 상태에서는 OFF가 거부된다")
# ============================================================
press_key(c, closed=True)
check("여전히 조그 활성", c.active is True)
check("복귀 시작 안 함", c.releasing is False)
check("오프셋 유지", c.offset[jc.IDX_UD] != 0)


# ============================================================
print("\n[7] 물체를 놓은 뒤 OFF -> 서서히 0으로 복귀")
# ============================================================
press_key(c, closed=False)
check("복귀 시작", c.releasing is True)

mid = run(c, 5)  # 복귀 도중
check("한 번에 0이 되지 않는다", mid[jc.IDX_UD] != 0, mid[jc.IDX_UD])

off = run(c, 40)  # 복귀 완료
check("오프셋 0 도달", off[jc.IDX_UD] == 0 and off[jc.IDX_LR] == 0, off)
check("조그 비활성", c.active is False)
check("복귀 플래그 해제", c.releasing is False)


# ============================================================
print("\n[8] 인터록 - 조그 중 토글이 켜지면 강제 해제")
# ============================================================
c = JogController("R")
press_key(c)
run(c, 100, jy=PUSH_MAX)
check("조그 활성 확인", c.active is True)

run(c, 1, gate=1)  # 토글 ON
check("강제 복귀 시작", c.releasing is True)

off = run(c, 40, gate=1)
check("오프셋 0 도달", off[jc.IDX_UD] == 0, off[jc.IDX_UD])


# ============================================================
print("\n[9] 래치 무효화 - 키를 누른 채 토글을 내려도 안 켜진다")
# ============================================================
c = JogController("R")
run(c, 5, gate=1, key7=1)  # 토글 ON + 키 누른 상태
check("토글 ON 중엔 조그 안 켜짐", c.active is False)

run(c, 5, gate=0, key7=1)  # 키를 계속 누른 채 토글만 내림
check("가짜 상승엣지 없음", c.active is False, "여기서 켜지면 버그")

run(c, 2, gate=0, key7=0)  # 키를 뗐다가
run(c, 2, gate=0, key7=1)  # 다시 누름
check("다시 누르면 정상 동작", c.active is True)


# ============================================================
print("\n[10] force_reset - 즉시 초기화")
# ============================================================
run(c, 100, jx=PUSH_MAX)
check("좌우 오프셋 존재", c.offset[jc.IDX_LR] != 0)
c.force_reset("그리퍼 교체")
check("즉시 0", c.is_offset_zero())
check("조그 비활성", c.active is False)


# ============================================================
print("\n[11] 부호 반전 확인 - 반대 방향으로 밀면 반대로 누적")
# ============================================================
c = JogController("L")
press_key(c)
off = run(c, 100, jy=PUSH_MIN)
check("음수 방향 누적", off[jc.IDX_UD] < 0, off[jc.IDX_UD])


# ============================================================
print("\n" + "=" * 46)
print("  통과 %d  /  실패 %d" % (_passed, _failed))
print("=" * 46)
if _failed == 0:
    print("  모든 검증 통과. 실기 투입 가능.\n")
else:
    print("  실패 항목을 먼저 수정할 것.\n")
