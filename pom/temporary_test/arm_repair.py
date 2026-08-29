"""
adc_check.py
─────────────────────────────────────────────────────────────
마스터암 ADC 전 채널 점검 (왼팔 + 오른팔)

두 가지를 본다.

  1) 지터 — 손을 뗀 정지 상태에서 값이 얼마나 흔들리는가
  2) 응답성 — 관절을 움직였을 때 그 채널이 실제로 반응하는가

2번이 핵심이다. 서보는 멀쩡한데 로봇팔이 안 움직인다면
해당 채널의 포텐셔미터가 값을 안 내보내고 있을 가능성이 크다.

사용법:
    python adc_check.py
"""

import csv
import statistics
import struct
import sys
import time
from datetime import datetime

import serial

# ═══════════════════════════════════════════════════════════
#  ▼ 실행 전 확인
# ═══════════════════════════════════════════════════════════

PORT = "COM13"
BAUDRATE = 115200

PACKET_STRUCT = "<2sBH16H5HBBBH"
PACKET_HEADER = b"\xaa\x55"

ADC_MAX = 4095  # 12비트 ADC 상한

JITTER_SECONDS = 12.0  # 지터 측정 시간
RESPONSE_SECONDS = 6.0  # 채널당 응답 확인 시간

# 현재 데드존 설정 (비교용)
DEADZONE_ENTER = 12
DEADZONE_EXIT = 20

# 응답으로 인정할 최소 변화량
RESPONSE_MIN = 150

# ═══════════════════════════════════════════════════════════

PACKET_SIZE = struct.calcsize(PACKET_STRUCT)

# mux 채널 → 사람이 읽는 이름
CHANNELS = {}
for i in range(1, 8):
    CHANNELS[i] = f"왼팔 {i}번"
for i in range(9, 16):
    CHANNELS[i] = f"오른팔 {i-8}번"


def hr(c="─", n=64):
    print(c * n)


def step(t):
    print()
    hr()
    print(f"  {t}")
    hr()


class Reader:
    def __init__(self):
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
        except Exception as e:
            sys.exit(f"[에러] {PORT} 열기 실패: {e}")
        self.buf = b""
        self.parsed = 0
        self.bad = 0
        self.rejected = 0
        print(f"  [연결] {PORT}")

    def collect(self, seconds, label=""):
        """지정 시간 동안 수집해서 채널별 값 리스트를 반환."""
        samples = {ch: [] for ch in CHANNELS}
        t0 = time.time()
        last = 0
        while time.time() - t0 < seconds:
            self.buf += self.ser.read(64)
            while True:
                i = self.buf.find(PACKET_HEADER)
                if i < 0 or len(self.buf) - i < PACKET_SIZE:
                    break
                chunk = self.buf[i : i + PACKET_SIZE]
                self.buf = self.buf[i + PACKET_SIZE :]
                try:
                    f = struct.unpack(PACKET_STRUCT, chunk)
                except struct.error:
                    self.bad += 1
                    continue

                # ── 패킷 검증 ──────────────────────────
                # 12비트 ADC 이므로 모든 채널이 0~4095 여야 한다.
                # 범위를 벗어나면 가짜 헤더에 동기화된 것이므로 버린다.
                mux = f[3:19]
                if any(v > ADC_MAX for v in mux):
                    self.rejected += 1
                    # 1바이트만 버리고 다시 헤더를 찾는다 (재동기화)
                    self.buf = chunk[1:] + self.buf
                    continue

                for ch in CHANNELS:
                    samples[ch].append(f[3 + ch])
                self.parsed += 1
            el = time.time() - t0
            if el - last >= 0.5:
                last = el
                print(f"    {label}{el:.0f}/{seconds:.0f}초", end="\r")
        print(" " * 60, end="\r")
        return samples

    def close(self):
        self.ser.close()


def main():
    print()
    hr("═")
    print("  마스터암 ADC 전 채널 점검")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    hr("═")

    r = Reader()
    rows = []

    try:
        # ═══ 1. 지터 ═══════════════════════════════════
        step("1. 지터 측정 — 정지 상태")
        print("  마스터암에서 손을 완전히 떼고, 진동이 없는 상태로 두세요.")
        input("  준비되면 Enter: ")

        s = r.collect(JITTER_SECONDS, "측정 중... ")

        if r.parsed == 0:
            print(f"\n  ⚠ 패킷을 못 읽었습니다 (파싱 실패 {r.bad}건)")
            print(f"     PACKET_STRUCT = {PACKET_STRUCT} ({PACKET_SIZE}바이트)")
            print("     펌웨어 패킷 포맷이 바뀌었으면 수정하세요.")
            return

        print(
            f"\n  패킷 {r.parsed}개  (파싱 실패 {r.bad}건, "
            f"범위이탈 폐기 {r.rejected}건)"
        )
        if r.rejected:
            ratio = r.rejected / (r.parsed + r.rejected) * 100
            print(f"  ⚠ 폐기율 {ratio:.1f}% — 가짜 헤더 동기화가 있었습니다.")
            print("     폐기율이 5%% 를 넘으면 통신 자체를 먼저 점검하세요.")
        print()
        print(f"  {'채널':<14s}{'평균':>7s}{'변동폭':>8s}{'표준편차':>10s}   판정")
        hr()

        jitter = {}
        for ch, name in CHANNELS.items():
            v = s[ch]
            if not v:
                continue
            rng = max(v) - min(v)
            sd = statistics.pstdev(v) if len(v) > 1 else 0.0
            avg = sum(v) / len(v)
            jitter[ch] = rng

            if rng >= DEADZONE_EXIT:
                mark = "⚠ 데드존 초과 — 자발 진동 발생"
            elif rng >= DEADZONE_ENTER:
                mark = "△ 데드존 경계"
            else:
                mark = "정상"
            print(f"  {name:<14s}{avg:>7.0f}{rng:>8d}{sd:>10.2f}   {mark}")
            rows.append(["jitter", name, round(avg), rng, round(sd, 2)])

        hr()
        over = [CHANNELS[c] for c, v in jitter.items() if v >= DEADZONE_EXIT]
        near = [
            CHANNELS[c]
            for c, v in jitter.items()
            if DEADZONE_ENTER <= v < DEADZONE_EXIT
        ]

        if over:
            print(f"  데드존 초과 ({len(over)}개): {', '.join(over)}")
            print("    → 이 관절들은 가만히 둬도 혼자 움직입니다")
        if near:
            print(f"  데드존 경계 ({len(near)}개): {', '.join(near)}")
        if not over and not near:
            print("  ✓ 전 채널이 데드존 아래로 안정적입니다")

        alljit = [v for v in jitter.values()]
        if alljit and min(alljit) >= DEADZONE_ENTER:
            print("\n  ⚠ 전 채널이 함께 흔들립니다 — 공통 원인입니다")
            print("     개별 배선이 아니라 아래를 의심하세요")
            print("       · MUX 채널 전환 후 정착 시간 부족")
            print("       · STM32 ADC 샘플링 시간이 짧음")
            print("       · 전원/접지 노이즈")

        # ═══ 2. 응답성 ═════════════════════════════════
        step("2. 응답성 확인 — 채널이 실제로 반응하는가")
        print("  관절을 하나씩 크게 움직이면서 해당 채널 값이 변하는지 봅니다.")
        print("  서보는 멀쩡한데 로봇팔이 안 움직이는 원인을 여기서 찾습니다.\n")
        print("  확인할 채널을 고르세요.")
        print("    a  = 전체 14채널 (오래 걸림)")
        print("    l  = 왼팔만")
        print("    rr = 오른팔만")
        print("    번호 직접 입력 (예: 11 또는 11,13)")
        print("    s  = 건너뛰기")

        sel = input("\n  선택: ").strip().lower()

        if sel == "s":
            targets = []
        elif sel == "a":
            targets = list(CHANNELS)
        elif sel == "l":
            targets = [c for c in CHANNELS if c <= 7]
        elif sel == "rr":
            targets = [c for c in CHANNELS if c >= 9]
        else:
            targets = []
            for tok in sel.replace(" ", "").split(","):
                if tok.isdigit() and int(tok) in CHANNELS:
                    targets.append(int(tok))

        dead = []
        for ch in targets:
            name = CHANNELS[ch]
            print(f"\n  ── {name} (mux{ch}) ──")
            input(f"     이 관절을 끝에서 끝까지 크게 움직일 준비가 되면 Enter: ")
            print("     지금 움직이세요!")
            s2 = r.collect(RESPONSE_SECONDS, "     ")
            v = s2[ch]
            if not v:
                print("     ⚠ 데이터 없음")
                continue
            rng = max(v) - min(v)
            print(f"     최소 {min(v)}   최대 {max(v)}   변화량 {rng}")
            if max(v) > ADC_MAX:
                print(f"     ⚠ ADC 상한({ADC_MAX})을 넘는 값이 있습니다 — 측정 무효")
            rows.append(["response", name, min(v), max(v), rng])

            if rng >= RESPONSE_MIN:
                print("     ✓ 정상 반응")
            elif rng >= DEADZONE_EXIT:
                print("     △ 조금만 반응 — 포텐셔미터 접촉이 불안정하거나")
                print("        가동범위가 좁게 물려 있을 수 있습니다")
                dead.append((name, ch, rng))
            else:
                print("     ✗ 거의 반응 없음")
                print("        → 이 채널이 죽어 있습니다. 서보가 아니라")
                print("           포텐셔미터/배선 문제입니다")
                dead.append((name, ch, rng))

        # ═══ 결론 ══════════════════════════════════════
        if targets:
            step("결론")
            if dead:
                print("  반응하지 않는 채널:")
                for name, ch, rng in dead:
                    print(f"    · {name} (mux{ch})   변화량 {rng}")
                print("\n  확인 순서")
                print("    1) 해당 포텐셔미터 3선(VCC/GND/신호) 재체결")
                print("    2) MUX 입력핀 점퍼선 접촉 확인")
                print("    3) 포텐셔미터 양단 저항을 테스터로 측정")
                print("       (축을 돌리며 값이 연속적으로 변하는지)")
                print("    4) 그래도 안 되면 포텐셔미터 교체")
                print("\n  ※ 서보는 정상인데 로봇팔이 안 움직였다면")
                print("     이것이 원인입니다.")
            else:
                print("  ✓ 확인한 모든 채널이 정상 반응합니다.")
                print("    → 입력은 멀쩡하므로 원인은 제어부 코드입니다.")
                print("       데드존 / MAX_DELTA / 이상탐지 FSM 을 확인하세요.")

    except KeyboardInterrupt:
        print("\n\n  [중단] 사용자 종료")
    finally:
        if rows:
            fn = f"adc_check_{datetime.now():%Y%m%d_%H%M}.csv"
            with open(fn, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["type", "channel", "a", "b", "range"])
                w.writerows(rows)
            print(f"\n  [저장] {fn}")
        r.close()
        print("  완료.")


if __name__ == "__main__":
    main()
