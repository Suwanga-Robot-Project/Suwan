import time
import csv
import sys
from scservo_sdk import *
import scservo_sdk as _scs

# ==========================================
# 1. API Auto-Detect & Wrapper Logic
# ==========================================
PROTOCOL_END = 0

def build_packet_handler(ph):
    factory = getattr(_scs, "PacketHandler", None)
    if factory is not None:
        try: return factory(PROTOCOL_END), "port_first"
        except TypeError:
            try: return factory(), "port_first"
            except TypeError: pass
    for _name in ("sms_sts", "sms_sms"):
        cls = getattr(_scs, _name, None)
        if cls is not None: return cls(ph), "port_bound"
    cls = getattr(_scs, "protocol_packet_handler", None)
    if cls is not None: return cls(ph, PROTOCOL_END), "port_bound"
    return None, None

def write1(ph, pkt, style, sid, addr, val):
    if style == "port_first": return pkt.write1ByteTxRx(ph, sid, addr, val)
    return pkt.write1ByteTxRx(sid, addr, val)

def write2(ph, pkt, style, sid, addr, val):
    if style == "port_first": return pkt.write2ByteTxRx(ph, sid, addr, val)
    return pkt.write2ByteTxRx(sid, addr, val)

# [FIX 1] TxOnly Wrapper to eliminate ACK waiting delays (Crucial for smooth playback)
def write2_only(ph, pkt, style, sid, addr, val):
    fn = getattr(pkt, "write2ByteTxOnly", None)
    if fn is not None:
        if style == "port_first": return fn(ph, sid, addr, val)
        return fn(sid, addr, val)
    else:
        # Fallback if TxOnly is not supported by SDK version
        return write2(ph, pkt, style, sid, addr, val)

# ==========================================
# 2. Configuration
# ==========================================
PORT_LEFT = '/dev/ttyACM0'
PORT_RIGHT = '/dev/ttyACM1'
BAUDRATE = 1000000

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41    # [FIX 2] Added Acceleration Register
ADDR_GOAL_POSITION = 42

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
PAN_ID = 22
TILT_ID = 33

print("=====================================================")
print(" Physical AI Autonomous Replay Agent (Optimized)")
print("=====================================================")

filename = input("Enter the CSV filename to playback (e.g., ai_datasets/demo_123.csv): ").strip()

# [FIX 3] Pre-load entire trajectory into RAM to avoid SD Card I/O stutters
trajectory_frames = []
try:
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader) # Skip Header
        for row in reader:
            timestamp = float(row[0])
            ticks = [int(v) for v in row[1:17]]
            trajectory_frames.append((timestamp, ticks))
    print(f"[LOAD] Successfully loaded {len(trajectory_frames)} frames into memory.")
except Exception as e:
    print(f"[ERROR] Could not open or parse file: {e}")
    sys.exit(1)

# Initialize Hardware
portHandler_left = PortHandler(PORT_LEFT)
packetHandler_left, style_left = build_packet_handler(portHandler_left)

portHandler_right = PortHandler(PORT_RIGHT)
packetHandler_right, style_right = build_packet_handler(portHandler_right)

portHandler_left.openPort(); portHandler_left.setBaudRate(BAUDRATE)
portHandler_right.openPort(); portHandler_right.setBaudRate(BAUDRATE)

# Apply Torque and Acceleration
for motor_id in MOTORS_LEFT: 
    write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_ACCELERATION, 50)
    write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_TORQUE_ENABLE, 1)

for motor_id in MOTORS_RIGHT: 
    write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_ACCELERATION, 50)
    write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_TORQUE_ENABLE, 1)

write1(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_ACCELERATION, 50)
write1(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_TORQUE_ENABLE, 1)
write1(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_ACCELERATION, 100)
write1(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_TORQUE_ENABLE, 1)

print("\n[AI] Starting Autonomous Replay in 3 seconds...")
time.sleep(3)

# ==========================================
# 3. Trajectory Execution Loop (Absolute Time Sync)
# ==========================================
try:
    frame_count = 0
    start_time = time.time() # [FIX 4] Absolute zero-point for perfect timing
    
    for timestamp, ticks in trajectory_frames:
        
        # Absolute wait prevents cumulative drift
        target_time = start_time + timestamp
        now = time.time()
        if target_time > now:
            time.sleep(target_time - now)
            
        # Execute Motors using TxOnly (Fire and forget = ultra fast)
        for i in range(7):
            write2_only(portHandler_left, packetHandler_left, style_left, MOTORS_LEFT[i], ADDR_GOAL_POSITION, ticks[i])
            write2_only(portHandler_right, packetHandler_right, style_right, MOTORS_RIGHT[i], ADDR_GOAL_POSITION, ticks[i+7])
        write2_only(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_GOAL_POSITION, ticks[14])
        write2_only(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_GOAL_POSITION, ticks[15])
        
        frame_count += 1
        print(f"\r[AI EXECUTING] Frame {frame_count}/{len(trajectory_frames)} applied...", end="")

    print("\n\n[SUCCESS] Autonomous task completed perfectly!")

except KeyboardInterrupt:
    print("\n[STOP] Replay interrupted by user.")
finally:
    for motor_id in MOTORS_LEFT: write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_TORQUE_ENABLE, 0)
    for motor_id in MOTORS_RIGHT + [PAN_ID, TILT_ID]: write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_TORQUE_ENABLE, 0)
    portHandler_left.closePort(); portHandler_right.closePort()
