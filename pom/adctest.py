import serial
import struct
import time

ser = serial.Serial("COM13", 115200, timeout=1)
PACKET_FORMAT = "<2sBH16H5HBBH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
buf = bytearray()

print("가변저항을 건드리지 말고 그대로 두세요. 5초간 측정합니다.")
start = time.time()
values = []

while time.time() - start < 5:
    data = ser.read(ser.in_waiting or 1)
    if not data:
        continue
    buf.extend(data)
    while len(buf) >= PACKET_SIZE:
        idx = buf.find(b"\xaa\x55")
        if idx == -1:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < PACKET_SIZE:
            break
        packet = bytes(buf[:PACKET_SIZE])
        del buf[:PACKET_SIZE]
        unpacked = struct.unpack(PACKET_FORMAT, packet)
        adc_ind = unpacked[19:24]
        values.append(adc_ind[4])

print(f"중립 상태 raw 값 범위: {min(values)} ~ {max(values)}")
print(f"평균: {sum(values)/len(values):.0f}")
