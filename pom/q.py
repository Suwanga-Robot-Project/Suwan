import serial
import struct
import time

PORT_ADC = "COM13"
BAUD_ADC = 115200

PACKET_FORMAT = "<2sBH16H4HBBH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


def calc_crc16_ccitt(data, initial=0xFFFF):
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            )
    return crc


ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
buf = bytearray()

packet_count = 0
crc_fail_count = 0
garbage_byte_count = 0
last_report = time.time()

prev_mux = None
change_count = [0] * 16

print("실행 중 — 관절을 하나씩 움직이면서 어떤 인덱스가 반응하는지 확인하세요.")
print("Ctrl+C로 종료")

try:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue
        buf.extend(data)

        while len(buf) >= PACKET_SIZE:
            idx = buf.find(b"\xaa\x55")
            if idx == -1:
                garbage_byte_count += len(buf)
                buf.clear()
                break
            if idx > 0:
                garbage_byte_count += idx
                del buf[:idx]
            if len(buf) < PACKET_SIZE:
                break

            packet_bytes = bytes(buf[:PACKET_SIZE])
            del buf[:PACKET_SIZE]

            unpacked = struct.unpack(PACKET_FORMAT, packet_bytes)
            seq_num = unpacked[2]
            mux_adc = unpacked[3:19]
            adc_ind = unpacked[19:23]
            sw0, sw1 = unpacked[23], unpacked[24]
            crc_recv = unpacked[25]

            crc_calc = calc_crc16_ccitt(packet_bytes[:-2])
            if crc_calc != crc_recv:
                crc_fail_count += 1
                continue

            packet_count += 1

            if prev_mux is not None:
                for i in range(16):
                    if mux_adc[i] != prev_mux[i]:
                        change_count[i] += 1
            prev_mux = mux_adc

            if time.time() - last_report > 1.0:
                elapsed = time.time() - last_report
                rate = packet_count / elapsed if elapsed > 0 else 0
                print(
                    f"--- {rate:.1f} pkt/s | CRC실패:{crc_fail_count} | 쓰레기바이트:{garbage_byte_count} ---"
                )
                print(f"mux_adc(원본, 왼팔=[1~7], 오른팔=[9~15]): {list(mux_adc)}")
                print(f"adc_ind: {list(adc_ind)}  sw0:{sw0} sw1:{sw1}  seq:{seq_num}")
                print(f"채널별 변화감지횟수(지난 1초): {change_count}")
                packet_count = 0
                crc_fail_count = 0
                garbage_byte_count = 0
                change_count = [0] * 16
                last_report = time.time()

except KeyboardInterrupt:
    pass

ser.close()
print("종료")
