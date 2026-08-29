"""
COM13(STM32 ADC/키캡 패킷)에서 실시간으로 키캡 상태를 읽어
그리퍼 자동교체 FSM을 구동하는 메인 실행 스크립트.

패킷 구조 (main.c의 AdcPacket_t, 52바이트, 리틀엔디안):
  header(2) + msg_type(1) + seq_num(2) + mux_adc(16*2) + adc_ind(5*2)
  + sw0_toggle(1) + sw1_toggle(1) + key_states(1) + crc(2)
  = struct 포맷 "<2sBH16H5HBBBH", 총 28개 필드, index 0~27

  key_states는 index 26. 비트0~6 = 키1~7, 펌웨어에서 이미 반전 처리됨
  (버튼 눌리면 해당 비트가 1) — Python에서 추가로 뒤집을 필요 없음.

⚠️ 엣지 감지: 버튼이 눌려있는 "동안" 계속 트리거되지 않도록, 이전 프레임과
   비교해서 "새로 눌린 순간"에만 FSM에 값을 전달함 (계속 눌려있으면 무시).
"""

import serial
import struct
import time

import tool_changer_fsm
import key_input_handler
import station_positions

PORT_KEYCAP = "COM13"
BAUD_KEYCAP = 115200

PACKET_HEADER = b"\xaa\x55"
PACKET_SIZE = 52
PACKET_STRUCT = struct.Struct("<2sBH16H5HBBBH")
KEY_STATES_INDEX = 26  # unpacked 튜플에서 key_states 위치


def calc_crc16_ccitt(data: bytes, initial=0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_keys(key_states_byte):
    """key_states 1바이트에서 1~5번 키 눌림 여부(bool) 추출."""
    key1 = bool(key_states_byte & (1 << 0))
    key2 = bool(key_states_byte & (1 << 1))
    key3 = bool(key_states_byte & (1 << 2))
    key4 = bool(key_states_byte & (1 << 3))
    key5 = bool(key_states_byte & (1 << 4))
    return key1, key2, key3, key4, key5


def main():
    try:
        ser = serial.Serial(PORT_KEYCAP, BAUD_KEYCAP, timeout=1)
    except Exception as e:
        print(f"시리얼 열기 실패({PORT_KEYCAP}):", e)
        return

    print(f"{PORT_KEYCAP} 열기 성공. 키캡 감지 시작 (Ctrl+C로 종료)\n")

    left_fsm = tool_changer_fsm.ArmSwapFSM("left")
    right_fsm = tool_changer_fsm.ArmSwapFSM("right")

    # 현재 팔 위치 — 실시간 present position 연동 전까지는 NEUTRAL을 기본값으로 사용
    # TODO: 실제로는 각 팔의 현재 조종 위치를 읽어서 넣는 게 정확함
    current_left_ticks = list(station_positions.NEUTRAL_TICKS_LEFT)
    current_right_ticks = list(station_positions.NEUTRAL_TICKS_RIGHT)

    # 엣지 감지용 — 직전 프레임의 눌림 상태
    prev_keys = (False, False, False, False, False)

    buf = bytearray()

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)

            while True:
                idx = buf.find(PACKET_HEADER)
                if idx == -1:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PACKET_SIZE:
                    break

                raw_packet = bytes(buf[:PACKET_SIZE])
                unpacked = PACKET_STRUCT.unpack(raw_packet)

                crc_calc = calc_crc16_ccitt(raw_packet[: PACKET_SIZE - 2])
                crc_received = unpacked[-1]
                if crc_calc != crc_received:
                    del buf[:2]
                    continue

                key_states_byte = unpacked[KEY_STATES_INDEX]
                current_keys = parse_keys(key_states_byte)

                # ===== 엣지 감지: 새로 눌린 키만 골라냄 =====
                # (직전엔 안 눌렸는데 이번엔 눌린 것만 True, 나머지는 False로)
                edge_keys = tuple(
                    now and not prev for now, prev in zip(current_keys, prev_keys)
                )
                prev_keys = current_keys

                if any(edge_keys):
                    key1, key2, key3, key4, key5 = edge_keys
                    left_target, right_target = key_input_handler.parse_key_input(
                        key1, key2, key3, key4, key5
                    )
                    if left_target is not None or right_target is not None:
                        print(
                            f"[키 눌림 감지] 1={key1} 2={key2} 3={key3} 4={key4} 5={key5} "
                            f"→ left={left_target}, right={right_target}"
                        )
                else:
                    left_target, right_target = None, None

                # ===== FSM 업데이트 (매 프레임 호출, 엣지 없으면 None으로 그냥 진행만) =====
                left_fsm.update(left_target, current_left_ticks)
                right_fsm.update(right_target, current_right_ticks)

                del buf[:PACKET_SIZE]

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n=== 종료 ===")


if __name__ == "__main__":
    main()
