"""
수완 로봇 - 조이스틱 미세조작(조그) 제어 모듈

키캡 7번을 누르면 마스터암 위 조이스틱이 미세조정 입력으로 바뀌고,
다시 누르면 오프셋이 0으로 복귀하여 1:1 매핑이 회복된다.

이 파일은 기존 제어 코드에 의존하지 않는 순수 로직 모듈이다.
서보에 직접 명령을 보내지 않으며, 오프셋 값만 계산해서 돌려준다.

사용 예:
    from jog_control import JogController

    jog_left = JogController('L')

    # 매 루프마다
    offset = jog_left.update(
        gate_toggle   = sw1_toggle,      # 왼팔은 바퀴 토글, 오른팔은 팬틸트 토글
        key7          = keys[6],
        joy_x         = ind2,            # 좌우
        joy_y         = ind3,            # 상하
        gripper_closed= gripper_is_closed_left,
        dt            = loop_dt,
    )
    final_ticks[i] = mapped_ticks[i] + offset[i]
"""

# ============================================================
#  튜닝 상수 - 실기에서 조정하는 값들
# ============================================================

IDX_UD = 0  # 조이스틱 상하가 움직일 관절 인덱스 (0 = 1번 모터, 어깨)
IDX_LR = 5  # 조이스틱 좌우가 움직일 관절 인덱스 (5 = 6번 모터, 손목)

K_UD = 5.0  # 상하 최대 이동 속도 [tick/s]  - 1번은 반경이 커서 작게
K_LR = 30.0  # 좌우 최대 이동 속도 [tick/s]  - 6번은 반경이 작아서 크게

CLAMP_UD = 25.0  # 상하 최대 보정량 [tick]  (약 ±15mm)
CLAMP_LR = 160.0  # 좌우 최대 보정량 [tick]  (약 ±15mm)

JOY_NEUTRAL = 2033  # 조이스틱 중립값 (실측)
JOY_DEADBAND = 200  # 중립 인근 무시 구간
JOY_SPAN = 2033  # 중립에서 끝까지의 폭

RELEASE_FRAMES = 20  # 오프셋을 0으로 되돌리는 데 걸리는 프레임 수 (약 0.4초)

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
        self.last_warning = None  # HUD에 띄울 경고 (1회성)

        self._key_prev = 0
        self._step_ud = CLAMP_UD / RELEASE_FRAMES
        self._step_lr = CLAMP_LR / RELEASE_FRAMES

    # --------------------------------------------------------
    #  매 루프마다 호출
    # --------------------------------------------------------
    def update(self, gate_toggle, key7, joy_x, joy_y, gripper_closed, dt):
        """
        gate_toggle    : 이 조이스틱을 쓰는 다른 기능의 토글 (왼팔=바퀴, 오른팔=팬틸트)
        key7           : 키캡 7번 현재 상태 (0 또는 1)
        joy_x, joy_y   : 조이스틱 원시 ADC 값 (좌우, 상하)
        gripper_closed : 그리퍼가 물체를 쥐고 있는 상태인지 (True/False)
        dt             : 이전 루프로부터 경과 시간 [초]

        반환: 길이 num_channels의 정수 오프셋 리스트
        """
        self.last_warning = None

        # --- 1. 인터록 -------------------------------------
        # 토글이 켜져 있으면 조이스틱은 바퀴/팬틸트 소유다. 조그는 성립하지 않는다.
        if gate_toggle:
            if self.active and not self.releasing:
                self.releasing = True
                self.last_warning = "토글 전환으로 미세조정 해제"
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
                elif gripper_closed:
                    # 물체를 쥔 채로 끄면 팔이 원위치로 돌아가며 물체를 놓친다
                    self.last_warning = "물체를 놓은 뒤 해제하세요"
                else:
                    # ON -> OFF (복귀 시작)
                    self.releasing = True
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
          - run_gripper_swap() 완료
          - FSM ERROR 진입
          - 종료 시퀀스 NEUTRAL 복귀 직전
        """
        self.offset = [0.0] * self.num_channels
        self.active = False
        self.releasing = False
        self._key_prev = 0
        if reason:
            self.last_warning = "미세조정 초기화 (%s)" % reason

    # --------------------------------------------------------
    #  HUD 표시용
    # --------------------------------------------------------
    def status_text(self):
        if not self.active:
            return "조그 OFF"
        state = "복귀중" if self.releasing else "ON"
        return "조그 %s  상하 %+d / 좌우 %+d" % (
            state,
            int(round(self.offset[IDX_UD])),
            int(round(self.offset[IDX_LR])),
        )

    def is_offset_zero(self):
        return all(abs(v) < 0.5 for v in self.offset)
