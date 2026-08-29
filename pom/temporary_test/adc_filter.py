"""
adc_filter.py
─────────────────────────────────────────────────────────────
마스터암 ADC 노이즈 제거 필터

N5.py(유선 직결)와 raspi2.py(라파 실전) 양쪽에서 동일하게 사용한다.
두 파일 모두 패킷 파싱 직후에 한 줄만 추가하면 된다.

    mux_adc_vals = adc_filter.apply(mux_adc_vals)

해결하는 문제
    가끔 튀는 스파이크 → 관절이 혼자 움찔거림

원리
    최근 N개 값 중 "가운데 값"을 쓴다.
    평균과 달리 중앙값은 튄 값 하나에 끌려가지 않는다.

        원본:   2660  2662  2690  2661  2659    ← 2690이 스파이크
        평균:   2666.4                          ← 끌려감
        중앙값:  2661                           ← 무시함  ✓

※ 이 모듈은 아무것도 출력하지 않는다.
  통계가 필요하면 report() / report_text() 가 문자열이나 dict 를
  돌려주므로, 호출부에서 원할 때만 print 하면 된다.

작성 2026-08-17
"""

from collections import deque

# ═══════════════════════════════════════════════════════════
#  설정
# ═══════════════════════════════════════════════════════════

ADC_MAX = 4095  # 12비트 ADC 상한
NUM_CHANNELS = 16

# 기본 창 크기 (홀수여야 중앙값이 하나로 정해진다)
#   5 → 지연 약 2프레임(42ms). 사람이 체감 못 함
DEFAULT_WINDOW = 5

# 채널별 창 크기 예외
#   접촉 불량이 심한 채널은 창을 키운다. 지연이 늘지만 안정성이 우선.
#   ⚠ 하드웨어 조치가 끝나면 이 항목을 지우고 기본값으로 되돌릴 것.
CHANNEL_WINDOW = {
    11: 9,  # 오른팔 3번 — 서보 전원 노이즈 + 접촉 불량
}

# '큰 보정'으로 볼 기준 (tick)
BIG_CORRECTION = 15

# 채널 번호 → 사람이 읽는 이름 (통계용)
CHANNEL_NAME = {}
for _i in range(1, 8):
    CHANNEL_NAME[_i] = f"왼팔 {_i}번"
for _i in range(9, 16):
    CHANNEL_NAME[_i] = f"오른팔 {_i - 8}번"


# ═══════════════════════════════════════════════════════════
#  필터
# ═══════════════════════════════════════════════════════════


class AdcFilter:
    """16채널 ADC 값에 채널별 중앙값 필터를 적용한다.

    상태를 갖고 있으므로 인스턴스를 하나 만들어 계속 재사용한다.
    팔별로 나눌 필요 없다 — 채널 번호로 이미 구분된다.
    """

    def __init__(self, window=DEFAULT_WINDOW, channel_window=None):
        self.default_window = window
        self.channel_window = dict(CHANNEL_WINDOW)
        if channel_window:
            self.channel_window.update(channel_window)

        self.hist = {}
        for ch in range(NUM_CHANNELS):
            w = self.channel_window.get(ch, self.default_window)
            self.hist[ch] = deque(maxlen=w)

        # 통계 (진단용 — 출력은 호출부에서)
        self.frames = 0
        self.clamped = {ch: 0 for ch in range(NUM_CHANNELS)}
        self.corrected = {ch: 0 for ch in range(NUM_CHANNELS)}
        self.correction_sum = {ch: 0 for ch in range(NUM_CHANNELS)}
        self.big_corrections = {ch: 0 for ch in range(NUM_CHANNELS)}
        self.max_correction = {ch: 0 for ch in range(NUM_CHANNELS)}
        self.last_raw = [0] * NUM_CHANNELS
        self.last_out = [0] * NUM_CHANNELS

    # ───────────────────────────────────────────────────────
    def reset(self):
        """필터 이력을 비운다.

        호출해야 하는 지점:
          - system_ready 직후 / 메인 루프 진입 전
          - 통신 재연결 직후
        """
        for ch in range(NUM_CHANNELS):
            self.hist[ch].clear()

    # ───────────────────────────────────────────────────────
    def apply(self, mux_adc):
        """16채널 원본 값을 받아 필터링된 값을 돌려준다.

        mux_adc : 길이 16의 시퀀스 (list/tuple)
        반환    : 길이 16의 list

        어떤 경우에도 예외를 던지거나 출력하지 않는다.
        입력이 이상하면 원본을 그대로 돌려준다.
        """
        try:
            if mux_adc is None or len(mux_adc) < NUM_CHANNELS:
                return list(mux_adc) if mux_adc else [0] * NUM_CHANNELS
        except TypeError:
            return [0] * NUM_CHANNELS

        self.frames += 1
        out = [0] * NUM_CHANNELS

        for ch in range(NUM_CHANNELS):
            raw = mux_adc[ch]
            self.last_raw[ch] = raw

            # 범위 밖 값은 잘라낸다 (통신 오류/파싱 어긋남 대비)
            if raw < 0:
                raw = 0
                self.clamped[ch] += 1
            elif raw > ADC_MAX:
                raw = ADC_MAX
                self.clamped[ch] += 1

            h = self.hist[ch]
            h.append(raw)

            # 창이 다 안 찼어도 지금까지 값으로 중앙값을 낸다
            s = sorted(h)
            med = s[len(s) // 2]

            d = abs(med - raw)
            if d:
                self.corrected[ch] += 1
                self.correction_sum[ch] += d
                if d > BIG_CORRECTION:
                    self.big_corrections[ch] += 1
                if d > self.max_correction[ch]:
                    self.max_correction[ch] = d

            out[ch] = med
            self.last_out[ch] = med

        return out

    # ───────────────────────────────────────────────────────
    def status_line(self, ch):
        """한 채널의 현재 상태를 한 줄 문자열로 돌려준다 (출력 안 함)."""
        name = CHANNEL_NAME.get(ch, f"mux{ch}")
        w = self.channel_window.get(ch, self.default_window)
        return (
            f"{name:<12s} raw {self.last_raw[ch]:5d} → "
            f"out {self.last_out[ch]:5d}   창{w}"
        )

    # ───────────────────────────────────────────────────────
    def stats(self):
        """채널별 통계를 dict 로 돌려준다 (출력 안 함).

        반환: {채널번호: {"name", "window", "avg", "big_pct", "max", "verdict"}}

        avg(평균 보정량) 판정 기준
            3 미만  정상
            3~10    주의
            10 이상 배선/전원 점검
        """
        result = {}
        if self.frames == 0:
            return result

        for ch in sorted(CHANNEL_NAME):
            avg = self.correction_sum[ch] / self.frames
            if avg >= 10:
                verdict = "점검"
            elif avg >= 3:
                verdict = "주의"
            else:
                verdict = "정상"
            result[ch] = {
                "name": CHANNEL_NAME[ch],
                "window": self.channel_window.get(ch, self.default_window),
                "avg": avg,
                "big_pct": self.big_corrections[ch] / self.frames * 100,
                "max": self.max_correction[ch],
                "clamped": self.clamped[ch],
                "verdict": verdict,
            }
        return result

    # ───────────────────────────────────────────────────────
    def report_text(self):
        """통계표를 문자열로 만들어 돌려준다 (출력 안 함).

        보고 싶을 때만 호출부에서:
            print(adc_filter.report_text())

        ⚠ 보정 "횟수"는 진단에 못 쓴다. 중앙값 필터는 값이 ±1만
          흔들려도 최신값과 다른 결과를 낸다. 의미가 있는 건 크기다.
        ⚠ 측정 중 마스터암을 만지면 그 움직임도 보정량에 잡힌다.
          손을 완전히 뗀 상태에서 재야 한다.
        """
        if self.frames == 0:
            return "[ADC필터] 아직 처리한 프레임이 없습니다"

        lines = []
        bar = "─" * 66
        lines.append("")
        lines.append(bar)
        lines.append(f"  ADC 필터 통계 ({self.frames} 프레임)")
        lines.append(bar)
        lines.append(
            f"  {'채널':<12s}{'창':>4s}{'평균보정':>10s}"
            f"{'큰보정':>9s}{'최대보정':>10s}   판정"
        )
        lines.append(bar)

        worst = []
        for ch, s in self.stats().items():
            if s["verdict"] == "점검":
                worst.append((s["name"], s["avg"]))
            lines.append(
                f"  {s['name']:<12s}{s['window']:>4d}{s['avg']:>10.1f}"
                f"{s['big_pct']:>8.1f}%{s['max']:>10d}   {s['verdict']}"
            )

        lines.append(bar)
        lines.append("  평균보정 = 필터가 원본을 고친 평균 크기 (tick)")
        lines.append("             3 미만 정상 / 3~10 주의 / 10 이상 점검")
        lines.append(f"  큰보정   = {BIG_CORRECTION} tick 넘게 고친 프레임 비율")
        lines.append("  최대보정 = 한 번에 가장 크게 고친 값")

        if worst:
            lines.append("")
            lines.append("  점검 대상:")
            for name, avg in sorted(worst, key=lambda x: -x[1]):
                lines.append(f"    · {name}  (평균 {avg:.1f} tick)")
        lines.append("")
        return "\n".join(lines)

    # ───────────────────────────────────────────────────────
    def report(self):
        """호환용 — 아무것도 출력하지 않고 통계 문자열만 돌려준다.

        기존 코드의 adc_filter.report() 호출은 그대로 둬도 되며,
        이제 화면에 아무것도 찍히지 않는다.
        보고 싶으면  print(adc_filter.report())  로 바꾸면 된다.
        """
        return self.report_text()


# ═══════════════════════════════════════════════════════════
#  통신 지연 설정 (참고)
# ═══════════════════════════════════════════════════════════
#
#   2026-08-17 측정 결과, 모터 7개 연속 읽기에서
#
#       지연 없음 : 1회전  4.0 ms   성공률 100%
#       지연 8 ms : 1회전 68.1 ms   성공률 100%
#
#   17배 느려지는데 성공률 이득이 없으므로,
#   move_to_station.py 의 INTER_MOTOR_DELAY 를
#   0.008 → 0.001 로 줄이거나 제거해도 된다.
#
INTER_MOTOR_DELAY_RECOMMENDED = 0.001
