"""
수완 로봇 - 조이스틱 미세조작(조그) 제어 모듈

키캡 7번을 누르면 마스터암 위 조이스틱이 미세조정 입력으로 바뀌고,
다시 누르면 오프셋이 0으로 복귀하여 1:1 매핑이 회복된다.

이 파일은 기존 제어 코드에 의존하지 않는 순수 로직 모듈이다.
서보에 직접 명령을 보내지 않으며, 오프셋 값만 계산해서 돌려준다.

사용 예:
    import jog_control as jc
    from jog_control import JogController

    jog_left = JogController('L')

    # 매 루프마다
    offset = jog_left.update(
        gate_toggle   = sw1_toggle,      # 왼팔은 바퀴 토글, 오른팔은 팬틸트 토글
        key7          = keys[6],
        joy_x         = ind2,            # 좌우
        joy_y         = ind3,            # 상하
        gripper_closed= jc.is_gripper_closed(
            gripper_tick, GRIPPER_POS_OPEN, GRIPPER_POS_CLOSE
        ),
        dt            = loop_dt,
    )
    final_ticks[i] = mapped_ticks[i] + offset[i]

── 2026-08-24 수정 ──────────────────────────────────────────
조그가 한 번 켜지면 다시 꺼지지 않는 문제를 고쳤다. 원인 두 가지:

 1) gripper_closed 판정이 "완전히 열려 있지 않음"이었다.
    호출부가 (tick < MAX_OPEN - 300)으로 계산했는데, 오른팔 기준
    3635 미만이면 전부 파지로 잡혔다. 조종 중 대부분이 여기 걸린다.
    → is_gripper_closed()로 "조임 쪽 끝에 가까운지"를 보도록 바꿈.

 2) 파지 중 해제를 완전히 막고 있었다. 물체를 놓으려면 그리퍼를
    벌려야 하는데 조그가 안 꺼지니 빠져나갈 길이 없었다.
    → 첫 시도는 경고, 한 번 더 누르면 해제하는 2단계로 바꿈.

 3) last_warning을 매 프레임 None으로 지우고 HUD에도 안 띄워서,
    왜 안 꺼지는지 알 방법이 없었다.
    → 경고를 약 2초간 유지하고 status_text()에 노출.
"""

# ============================================================
#  튜닝 상수 - 실기에서 조정하는 값들
# ============================================================

IDX_UD = 0  # 조이스틱 상하가 움직일 관절 인덱스 (0 = 1번 모터, 어깨)
IDX_LR = 5  # 조이스틱 좌우가 움직일 관절 인덱스 (5 = 6번 모터, 손목)

K_UD = 40.0  # 상하 최대 속도 [tick/s] — 1초에 눈에 보이게 움직임
K_LR = 80.0  # 좌우 최대 속도 [tick/s]

CLAMP_UD = 200.0  # 상하 최대 보정량 [tick]
CLAMP_LR = 400.0  # 좌우 최대 보정량 [tick]

JOY_NEUTRAL = 2033  # 조이스틱 중립값 (실측)
JOY_DEADBAND = 200  # 중립 인근 무시 구간
JOY_SPAN = 2033  # 중립에서 끝까지의 폭

RELEASE_FRAMES = 40  # 오프셋을 0으로 되돌리는 프레임 수 (약 0.8초)

# 그리퍼가 "물체를 쥔 상태"로 볼 기준.
# 전체 가동폭에서 조임 끝으로부터 이 비율 안쪽이면 파지로 본다.
# 0.35 = 조임 끝에서 35% 이내
GRIPPER_CLOSED_RATIO = 0.35

# 경고를 HUD에 유지할 프레임 수 (루프 0.02s 기준 약 2초)
WARNING_FRAMES = 100

NUM_CHANNELS = 7  # 팔 하나당 채널 수

# 팔별 / 축별 부호.
# 양팔은 물리적으로 거울상이므로 같은 tick 방향이 반대 물리 방향이 될 수 있다.
# 실기에서 조이스틱을 밀어보고 반대로 가면 해당 값을 -1로 바꾼다.
SIGN = {
    "L": {IDX_UD: +1, IDX_LR: +1},
    "R": {IDX_UD: +1, IDX_LR: +1},
}


# ============================================================
#  내부 헬퍼
# ============================================================


def _normalize(raw):
    """조이스틱 원시값을 -1.0 ~ +1.0 으로 정규화. 데드밴드 안이면 0."""
    d = raw - JOY_NEUTRAL
    if abs(d) <= JOY_DEADBAND:
        return 0.0
    span = JOY_SPAN - JOY_DEADBAND
    v = (abs(d) - JOY_DEADBAND) / span
    if v > 1.0:
        v = 1.0
    return v if d > 0 else -v


def _clamp(v, limit):
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v


def _toward_zero(v, step):
    if v > step:
        return v - step
    if v < -step:
        return v + step
    return 0.0


# ============================================================
#  파지 판정 (호출부에서 사용)
# ============================================================


def is_gripper_closed(tick, pos_open, pos_close, ratio=GRIPPER_CLOSED_RATIO):
    """
    그리퍼가 '물체를 쥔 상태'인지 판정한다.

    tick      : 현재 그리퍼 목표 tick
    pos_open  : 완전 개방 tick (GRIPPER_POS_OPEN_*)
    pos_close : 완전 조임 tick (GRIPPER_POS_CLOSE_*)
    ratio     : 조임 끝에서 전체 폭의 이 비율 안쪽이면 파지로 본다

    ⚠️ "완전히 열려 있지 않다"로 판정하면 안 된다. 조종 중에는 대부분
       완전 개방 상태가 아니므로 거의 항상 파지로 잡히고, 그러면 조그를
       끌 수 없게 된다. 조임 쪽 끝에 가까운지를 봐야 한다.

    pos_open > pos_close 든 그 반대든 상관없이 동작한다.
    """
    if tick is None:
        return False

    span = pos_open - pos_close
    if span == 0:
        return False

    frac = (tick - pos_close) / float(span)  # 0=완전조임, 1=완전개방
    return frac < ratio


# ============================================================
#  조그 컨트롤러
# ============================================================


class JogController:
    """팔 하나의 조그 상태를 관리한다. 왼팔/오른팔 각각 한 개씩 만든다."""

    def __init__(self, arm, num_channels=NUM_CHANNELS):
        if arm not in ("L", "R"):
            raise ValueError("arm must be 'L' or 'R'")
        self.arm = arm
        self.num_channels = num_channels

        self.offset = [0.0] * num_channels  # 누적 보정값 (float로 유지)
        self.active = False  # 조그 모드 ON 여부
        self.releasing = False  # 0으로 복귀하는 중인지
        self.last_warning = None  # HUD에 띄울 경고

        self._key_prev = 0
        self._off_attempts = 0  # 파지 중 해제 시도 횟수
        self._warn_frames = 0  # 경고 잔여 표시 프레임
        self._step_ud = CLAMP_UD / RELEASE_FRAMES
        self._step_lr = CLAMP_LR / RELEASE_FRAMES

    # --------------------------------------------------------
    #  경고 표시
    # --------------------------------------------------------
    def _set_warning(self, text, frames=WARNING_FRAMES):
        """경고를 일정 프레임 동안 유지한다. 한 프레임만 띄우면 안 보인다."""
        self.last_warning = text
        self._warn_frames = frames

    def _tick_warning(self):
        if self._warn_frames > 0:
            self._warn_frames -= 1
            if self._warn_frames == 0:
                self.last_warning = None

    # --------------------------------------------------------
    #  매 루프마다 호출
    # --------------------------------------------------------
    def update(self, gate_toggle, key7, joy_x, joy_y, gripper_closed, dt):
        """
        gate_toggle    : 이 조이스틱을 쓰는 다른 기능의 토글 (왼팔=바퀴, 오른팔=팬틸트)
        key7           : 키캡 7번 현재 상태 (0 또는 1)
        joy_x, joy_y   : 조이스틱 원시 ADC 값 (좌우, 상하)
        gripper_closed : 물체를 쥐고 있는지 — is_gripper_closed()로 계산해서 넘길 것
        dt             : 이전 루프로부터 경과 시간 [초]

        반환: 길이 num_channels의 정수 오프셋 리스트
        """
        self._tick_warning()

        # --- 1. 인터록 -------------------------------------
        # 토글이 켜져 있으면 조이스틱은 바퀴/팬틸트 소유다. 조그는 성립하지 않는다.
        if gate_toggle:
            if self.active and not self.releasing:
                self.releasing = True
                self._off_attempts = 0
                self._set_warning("토글 전환으로 미세조정 해제")
            # 래치 무효화: 현재 키 상태를 그대로 기록해서 가짜 상승엣지를 막는다.
            # (키를 누른 채로 토글을 내렸을 때 갑자기 켜지는 것을 방지)
            self._key_prev = 1 if key7 else 0

        # --- 2. 키캡 7번 상승엣지 ---------------------------
        else:
            k = 1 if key7 else 0
            if k == 1 and self._key_prev == 0:
                if not self.active:
                    # OFF -> ON
                    self.active = True
                    self.releasing = False
                    self._off_attempts = 0
                    self._set_warning(None, 0)

                elif self.releasing:
                    # 복귀 중에 다시 누르면 해제를 취소하고 ON 유지.
                    # (실수로 껐을 때 되돌릴 방법이 있어야 한다)
                    self.releasing = False
                    self._off_attempts = 0
                    self._set_warning("해제 취소 — 미세조정 유지")

                elif gripper_closed and self._off_attempts == 0:
                    # 물체를 쥔 채로 끄면 팔이 원위치로 돌아가며 물체를 놓친다.
                    # 단 여기서 완전히 막으면 해제할 방법이 없어진다 —
                    # 물체를 놓으려면 그리퍼를 벌려야 하는데 조그가 안 꺼지므로.
                    # 그래서 첫 시도는 경고만, 한 번 더 누르면 해제한다.
                    self._off_attempts = 1
                    self._set_warning("파지 중 — 한 번 더 누르면 해제")

                else:
                    # ON -> OFF (복귀 시작)
                    self.releasing = True
                    self._off_attempts = 0

            self._key_prev = k

        # --- 3. 오프셋 누적 ---------------------------------
        if self.active and not self.releasing:
            nx = _normalize(joy_x)
            ny = _normalize(joy_y)
            if nx != 0.0:
                self.offset[IDX_LR] += nx * K_LR * SIGN[self.arm][IDX_LR] * dt
            if ny != 0.0:
                self.offset[IDX_UD] += ny * K_UD * SIGN[self.arm][IDX_UD] * dt
            self.offset[IDX_UD] = _clamp(self.offset[IDX_UD], CLAMP_UD)
            self.offset[IDX_LR] = _clamp(self.offset[IDX_LR], CLAMP_LR)

        # --- 4. 복귀 램프 -----------------------------------
        if self.releasing:
            self.offset[IDX_UD] = _toward_zero(self.offset[IDX_UD], self._step_ud)
            self.offset[IDX_LR] = _toward_zero(self.offset[IDX_LR], self._step_lr)
            if self.offset[IDX_UD] == 0.0 and self.offset[IDX_LR] == 0.0:
                self.releasing = False
                self.active = False

        return [int(round(v)) for v in self.offset]

    # --------------------------------------------------------
    #  즉시 리셋 (램프 없음)
    # --------------------------------------------------------
    def force_reset(self, reason=""):
        """
        아래 시점에서 호출한다:
          - system_ready 초기화
          - 그리퍼 완전 개방
          - run_gripper_swap() 시작/완료
          - FSM ERROR 진입
          - 종료 시퀀스 NEUTRAL 복귀 직전
        """
        self.offset = [0.0] * self.num_channels
        self.active = False
        self.releasing = False
        self._key_prev = 0
        self._off_attempts = 0
        if reason:
            self._set_warning("미세조정 초기화 (%s)" % reason)

    # --------------------------------------------------------
    #  HUD 표시용
    # --------------------------------------------------------
    def status_text(self):
        if not self.active:
            base = "조그 OFF"
        else:
            state = "복귀중" if self.releasing else "ON"
            base = "조그 %s  상하 %+d / 좌우 %+d" % (
                state,
                int(round(self.offset[IDX_UD])),
                int(round(self.offset[IDX_LR])),
            )
        if self.last_warning:
            base += "   [!] %s" % self.last_warning
        return base

    def is_offset_zero(self):
        return all(abs(v) < 0.5 for v in self.offset)


# ============================================================
#  자체 테스트 (하드웨어 없이 로직만 검증)
# ============================================================

if __name__ == "__main__":
    DT = 0.02
    passed = failed = 0

    def check(name, cond):
        global passed, failed
        if cond:
            passed += 1
            print(f"  PASS  {name}")
        else:
            failed += 1
            print(f"  FAIL  {name}")

    def press(jc_obj, gripper_closed=False, frames_held=2):
        """키캡 7번을 눌렀다 뗀다 (상승엣지 1회)."""
        for _ in range(frames_held):
            jc_obj.update(0, True, JOY_NEUTRAL, JOY_NEUTRAL, gripper_closed, DT)
        jc_obj.update(0, False, JOY_NEUTRAL, JOY_NEUTRAL, gripper_closed, DT)

    def idle(jc_obj, n, gripper_closed=False):
        for _ in range(n):
            jc_obj.update(0, False, JOY_NEUTRAL, JOY_NEUTRAL, gripper_closed, DT)

    print("=== is_gripper_closed 판정 ===")
    # 오른팔: OPEN=3935, CLOSE=0
    check("오른팔 완전개방(3935) → 파지 아님", not is_gripper_closed(3935, 3935, 0))
    check(
        "오른팔 3000 → 파지 아님 (기존 로직은 파지로 오판했음)",
        not is_gripper_closed(3000, 3935, 0),
    )
    check("오른팔 1000 → 파지", is_gripper_closed(1000, 3935, 0))
    check("오른팔 완전조임(0) → 파지", is_gripper_closed(0, 3935, 0))
    # 왼팔: OPEN=4100, CLOSE=500
    check("왼팔 3000 → 파지 아님", not is_gripper_closed(3000, 4100, 500))
    check("왼팔 1000 → 파지", is_gripper_closed(1000, 4100, 500))
    check("None → 파지 아님", not is_gripper_closed(None, 3935, 0))

    print("\n=== ON/OFF 토글 (빈손) ===")
    j = JogController("R")
    check("초기 OFF", not j.active)
    press(j)
    check("1회 누름 → ON", j.active and not j.releasing)
    press(j)
    # 오프셋이 0이면 되돌릴 게 없으므로 같은 프레임에 OFF까지 완료된다
    check("2회 누름 → OFF (오프셋 0이라 즉시 완료)", not j.active and not j.releasing)
    check("오프셋 0", j.is_offset_zero())

    # 오프셋이 있는 경우에는 램프를 거친다
    j2 = JogController("R")
    press(j2)
    for _ in range(50):
        j2.update(0, False, JOY_NEUTRAL + 1500, JOY_NEUTRAL, False, DT)
    press(j2)
    check("오프셋 있을 때 2회 누름 → 복귀 중", j2.releasing and j2.active)
    idle(j2, RELEASE_FRAMES + 5)
    check("램프 후 OFF", not j2.active and not j2.releasing)
    check("오프셋 0으로 복귀", j2.is_offset_zero())

    print("\n=== 파지 중 해제 (2단계) ===")
    j = JogController("R")
    press(j)
    # 조이스틱을 밀어 오프셋을 만든다
    for _ in range(50):
        j.update(0, False, JOY_NEUTRAL + 1500, JOY_NEUTRAL, False, DT)
    check("오프셋 생성됨", not j.is_offset_zero())

    press(j, gripper_closed=True)
    check("파지 중 1회 → 아직 ON (경고)", j.active and not j.releasing)
    check("경고 문구 표시됨", j.last_warning is not None)
    check("HUD에 경고 노출", "[!]" in j.status_text())

    press(j, gripper_closed=True)
    check("파지 중 2회 → 해제 시작", j.releasing)
    idle(j, RELEASE_FRAMES + 5, gripper_closed=True)
    check("복귀 완료 → OFF", not j.active)
    check("오프셋 0으로 복귀", j.is_offset_zero())

    print("\n=== 복귀 중 재누름 → 취소 ===")
    j = JogController("R")
    press(j)
    for _ in range(50):
        j.update(0, False, JOY_NEUTRAL + 1500, JOY_NEUTRAL, False, DT)
    press(j)
    check("복귀 중", j.releasing)
    idle(j, 5)
    press(j)
    check("재누름 → 해제 취소, ON 유지", j.active and not j.releasing)

    print("\n=== 토글 인터록 ===")
    j = JogController("R")
    press(j)
    check("ON 상태", j.active)
    j.update(1, False, JOY_NEUTRAL, JOY_NEUTRAL, False, DT)  # 토글 ON
    check("토글 켜짐 → 해제됨 (오프셋 0이라 즉시)", not j.active)

    # 오프셋이 있는 상태에서 토글이 켜지면 램프를 거친다
    jt = JogController("R")
    press(jt)
    for _ in range(50):
        jt.update(0, False, JOY_NEUTRAL + 1500, JOY_NEUTRAL, False, DT)
    jt.update(1, False, JOY_NEUTRAL, JOY_NEUTRAL, False, DT)
    check("오프셋 있을 때 토글 → 복귀 시작", jt.releasing)

    j2 = JogController("L")
    for _ in range(3):
        j2.update(1, True, JOY_NEUTRAL, JOY_NEUTRAL, False, DT)
    check("토글 중 키 누름 → 조그 안 켜짐", not j2.active)

    print("\n=== force_reset ===")
    j = JogController("R")
    press(j)
    for _ in range(50):
        j.update(0, False, JOY_NEUTRAL + 1500, JOY_NEUTRAL, False, DT)
    j.force_reset("그리퍼 교체")
    check("즉시 OFF", not j.active and not j.releasing)
    check("오프셋 즉시 0", j.is_offset_zero())
    check("초기화 경고 표시", j.last_warning is not None)

    print("\n=== 속도 / 상한 ===")
    j = JogController("R")
    press(j)
    for _ in range(50):  # 1초
        j.update(0, False, JOY_NEUTRAL, JOY_NEUTRAL + 1500, False, DT)
    ud = abs(j.offset[IDX_UD])
    check(f"1초 상하 이동 {ud:.0f}tick (K_UD={K_UD} 대비 타당)", 20 < ud <= K_UD)
    for _ in range(600):  # 충분히 오래
        j.update(0, False, JOY_NEUTRAL, JOY_NEUTRAL + 1500, False, DT)
    check(
        f"상한 도달 {abs(j.offset[IDX_UD]):.0f} == CLAMP_UD {CLAMP_UD}",
        abs(abs(j.offset[IDX_UD]) - CLAMP_UD) < 0.5,
    )

    print("\n=== 데드밴드 ===")
    j = JogController("R")
    press(j)
    for _ in range(50):
        j.update(0, False, JOY_NEUTRAL + 150, JOY_NEUTRAL + 150, False, DT)
    check("데드밴드 안에서는 오프셋 0", j.is_offset_zero())

    print()
    print("=" * 46)
    print(f"  통과 {passed}  /  실패 {failed}")
    print("=" * 46)
