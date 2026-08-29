"""
gripper_contact.py
─────────────────────────────────────────────────────────────
그리퍼 접촉 감지 정지 모듈

목적:
    그리퍼가 닫히는 중 서보의 Present Load를 읽어, 물체에 닿는 순간을
    감지해서 더 이상 조이지 않도록 목표값을 제한한다.
    물체의 크기/경도를 미리 몰라도 동작한다.

설계 원칙:
    1. 닫는 방향일 때만 폴링한다 (여는 중/정지 중에는 통신 부하 0)
    2. 연속 N프레임 초과일 때만 접촉으로 판정한다 (노이즈 오탐 방지)
    3. 접촉 후에도 여는 방향 명령은 항상 통과시킨다 (절대 잠기지 않음)
    4. 통신 실패 시 기능을 건너뛴다 (그리퍼를 절대 멈추지 않음)
    5. 임계값 미실측이면 조용히 비활성화된다 (에러로 죽지 않음)

작성 2026-08-17 / 실측 전 골격
"""

r
# ═══════════════════════════════════════════════════════════
#  레지스터
# ═══════════════════════════════════════════════════════════

# STS3215 Present Load (2바이트)
#   motor_stall_diagnostic.py 에서 이미 사용해 의미 있는 값을 얻은 주소.
#   하위 10비트 = 크기(0~1023), 비트10 = 방향
ADDR_PRESENT_LOAD = 60

LOAD_MAGNITUDE_MASK = 0x03FF
LOAD_DIRECTION_BIT = 0x0400


# ═══════════════════════════════════════════════════════════
#  그리퍼별 설정
# ═══════════════════════════════════════════════════════════

# (MAX_OPEN, MAX_CLOSE) — station_positions.py 실측값과 동일
GRIPPER_RANGE = {
    "default": (1172, 3769),
    "vise": (1404, 4001),
    "fine": (906, 3506),
    "nipper": (1208, 3083),
}

# ── 실측 후 여기를 채운다 ──────────────────────────────────
#    gripper_load_probe.py 실행 결과의 "권장 임계값"을 입력.
#    None 이면 해당 그리퍼는 접촉 감지가 자동으로 비활성화된다.
CONTACT_LOAD_THRESHOLD = {
    "default": None,
    "vise": None,
    "fine": None,
    "nipper": None,
}

# 이번 범위에서 제외할 그리퍼.
# 니퍼는 2단계 절단 설계가 확정된 뒤에 활성화한다.
# 활성화하려면 이 집합에서 "nipper" 를 빼기만 하면 된다.
EXCLUDED_GRIPPERS = {"nipper"}


# ═══════════════════════════════════════════════════════════
#  튜닝 파라미터
# ═══════════════════════════════════════════════════════════

CONTACT_CONSECUTIVE_FRAMES = 3  # 연속 몇 프레임 초과해야 접촉으로 볼지
CLOSE_MOVE_EPS = 2  # 이 tick 이상 변해야 "닫는 중"으로 판정
RELEASE_MARGIN = 15  # 접촉 지점보다 이만큼 열면 래치 해제
POLL_EVERY_N_FRAMES = 1  # 1=매 프레임. 루프가 느려지면 2로
MAX_READ_FAILURES = 20  # 연속 통신 실패가 이만큼이면 기능 자동 정지


# ═══════════════════════════════════════════════════════════
#  검출기
# ═══════════════════════════════════════════════════════════


class GripperContactDetector:
    """팔 하나의 그리퍼에 대한 접촉 감지기."""

    def __init__(
        self, arm_side, packet_handler, port_handler, gripper_motor_id, verbose=True
    ):
        """
        arm_side          : "left" / "right"  (로그용)
        packet_handler    : scservo_sdk 의 sms_sts 인스턴스
        port_handler      : 해당 팔의 PortHandler
        gripper_motor_id  : 왼팔 7, 오른팔 15
        """
        self.arm_side = arm_side
        self.ph = packet_handler
        self.port = port_handler
        self.gid = gripper_motor_id
        self.verbose = verbose

        self.enabled = True  # 통신 실패 누적 시 False 로 자동 전환
        self.latched = False  # 접촉 감지 상태
        self.contact_tick = None  # 접촉이 감지된 시점의 목표 tick
        self.over_count = 0  # 임계 초과 연속 프레임 수
        self.fail_count = 0  # 연속 통신 실패 수
        self.frame_count = 0
        self.last_load = 0  # 피드백(오디오/HUD)용 최신 부하값

    # ───────────────────────────────────────────────────────
    def reset(self, reason=""):
        """래치와 카운터를 초기화한다.

        호출해야 하는 지점:
          - system_ready 직후
          - 그리퍼 교체(run_gripper_swap) 완료 직후
          - 종료 시퀀스 진입 시
        """
        was_latched = self.latched
        self.latched = False
        self.contact_tick = None
        self.over_count = 0
        if was_latched and self.verbose:
            print(f"[접촉감지] {self.arm_side} 해제 ({reason})")

    # ───────────────────────────────────────────────────────
    def read_load(self):
        """Present Load 크기(0~1023)를 읽는다. 실패하면 None."""
        try:
            raw, comm, err = self.ph.read2ByteTxRx(
                self.port, self.gid, ADDR_PRESENT_LOAD
            )
        except Exception:
            self.fail_count += 1
            return None

        if comm != 0 or err != 0:
            self.fail_count += 1
            if self.fail_count >= MAX_READ_FAILURES:
                self.enabled = False
                print(
                    f"[접촉감지] {self.arm_side} 통신 실패 누적 "
                    f"{self.fail_count}회 — 기능을 정지합니다"
                )
            return None

        self.fail_count = 0
        magnitude = raw & LOAD_MAGNITUDE_MASK
        self.last_load = magnitude
        return magnitude

    # ───────────────────────────────────────────────────────
    def apply(self, target_tick, prev_tick, gripper_name):
        """그리퍼 목표 tick 을 접촉 상태에 따라 보정해서 돌려준다.

        target_tick   : 이번 프레임에 계산된 그리퍼 목표 (매핑 완료 후)
        prev_tick     : 직전 프레임에 실제로 보낸 그리퍼 tick
        gripper_name  : "default" / "vise" / "fine" / "nipper" / None

        반환: 보정된 target_tick (int)

        ※ 항상 int 를 반환한다. 어떤 이유로든 판정이 불가능하면
          입력값을 그대로 돌려준다. 그리퍼가 멈추는 일은 없다.
        """
        # ── 비활성 조건들 ────────────────────────────────
        if not self.enabled:
            return target_tick
        if gripper_name is None:
            return target_tick
        if gripper_name in EXCLUDED_GRIPPERS:
            return target_tick

        threshold = CONTACT_LOAD_THRESHOLD.get(gripper_name)
        if threshold is None:
            return target_tick  # 미실측 → 조용히 통과

        rng = GRIPPER_RANGE.get(gripper_name)
        if rng is None:
            return target_tick
        open_tick, close_tick = rng

        # 닫는 방향의 부호 (+1: tick 증가가 조임 / -1: 감소가 조임)
        closing_sign = 1 if close_tick > open_tick else -1

        # ── 이미 접촉 래치가 걸린 상태 ───────────────────
        if self.latched:
            # 여는 방향으로 충분히 명령했으면 해제
            if (target_tick - self.contact_tick) * closing_sign < -RELEASE_MARGIN:
                self.reset("조종자가 폄")
                return target_tick
            # 아니면 접촉 지점보다 더 조이지 못하게 제한
            if (target_tick - self.contact_tick) * closing_sign > 0:
                return self.contact_tick
            return target_tick

        # ── 닫는 중인지 판정 ─────────────────────────────
        delta = (target_tick - prev_tick) * closing_sign
        if delta < CLOSE_MOVE_EPS:
            self.over_count = 0
            return target_tick  # 열거나 멈춰 있음 → 폴링 안 함

        # ── 폴링 주기 ────────────────────────────────────
        self.frame_count += 1
        if POLL_EVERY_N_FRAMES > 1 and (self.frame_count % POLL_EVERY_N_FRAMES):
            return target_tick

        load = self.read_load()
        if load is None:
            return target_tick  # 읽기 실패 → 이번 프레임은 그냥 통과

        # ── 접촉 판정 ────────────────────────────────────
        if load >= threshold:
            self.over_count += 1
            if self.over_count >= CONTACT_CONSECUTIVE_FRAMES:
                self.latched = True
                self.contact_tick = prev_tick  # 닿기 직전 위치에서 정지
                self.over_count = 0
                if self.verbose:
                    print(
                        f"[접촉감지] {self.arm_side} 접촉 "
                        f"(load={load}, tick={prev_tick}, {gripper_name})"
                    )
                return self.contact_tick
        else:
            self.over_count = 0

        return target_tick

    # ───────────────────────────────────────────────────────
    def status(self):
        """HUD / 오디오 피드백용 상태."""
        return {
            "arm": self.arm_side,
            "enabled": self.enabled,
            "contact": self.latched,
            "load": self.last_load,
        }


# ═══════════════════════════════════════════════════════════
#  오디오 피드백 (선택)
# ═══════════════════════════════════════════════════════════


def load_to_frequency(load, threshold, f_min=400, f_max=1200):
    """부하값을 비프 주파수로 변환한다."""
    if threshold is None or threshold <= 0:
        return f_min
    ratio = max(0.0, min(1.0, load / float(threshold)))
    return int(f_min + (f_max - f_min) * ratio)


def beep(freq, ms=40):
    """Windows 전용. 라파에서는 무시된다."""
    try:
        import winsound

        winsound.Beep(int(freq), int(ms))
    except Exception:
        pass
