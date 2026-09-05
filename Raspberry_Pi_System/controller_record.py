import socket
import time
import threading
import csv
import os
from gpiozero import OutputDevice, Button
import wheels_safe 

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

def read2(ph, pkt, style, sid, addr):
    if style == "port_first": return pkt.read2ByteTxRx(ph, sid, addr)
    return pkt.read2ByteTxRx(sid, addr)

# ==========================================
# 2. Main Configuration
# ==========================================
UDP_IP = "0.0.0.0"
UDP_PORT = 5009      
TCP_PORT = 5006  

LAPTOP_IP = "192.168.1.104"  
LOAD_PORT = 5009            
FEEDBACK_PORT = 5010        

PORT_LEFT = '/dev/ttyACM0'
PORT_RIGHT = '/dev/ttyACM1'
BAUDRATE = 1000000

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56  
ADDR_PRESENT_LOAD = 60  

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
PAN_ID = 22
TILT_ID = 33

left_opened = False
right_opened = False
is_running = True

latest_keys_str = "0000000"
latest_sw1 = 0
latest_sw2 = 0

feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# --- Lift Setup ---
lift_dir_1 = OutputDevice(17, initial_value=False)
lift_dir_2 = OutputDevice(27, initial_value=False)
limit_top = Button(23, pull_up=True)
limit_bottom = Button(24, pull_up=True)
current_lift_state = 0  
is_auto_changing = False  

def stop_lift():
    global current_lift_state
    lift_dir_1.off()
    lift_dir_2.off()
    current_lift_state = 0

def update_lift_motor(new_state):
    global current_lift_state
    if is_auto_changing or new_state == current_lift_state: return
    lift_dir_1.off(); lift_dir_2.off()
    time.sleep(0.05)
    if new_state == 1 and not limit_top.is_pressed:
        lift_dir_1.on(); current_lift_state = 1
    elif new_state == -1 and not limit_bottom.is_pressed:
        lift_dir_2.on(); current_lift_state = -1
    else: current_lift_state = 0

# ==========================================
# 3. AI Data Recorder Thread (NEW FEATURE)
# ==========================================
is_recording = False
record_file = None
csv_writer = None
record_start_time = 0.0

def recorder_command_thread():
    global is_recording, record_file, csv_writer, record_start_time
    print("\n=====================================================")
    print(" [PHYSICAL AI] Demonstration Data Logger is READY")
    print(" Type 'r' and press ENTER to START recording.")
    print(" Type 's' and press ENTER to STOP recording.")
    print("=====================================================\n")
    
    while True:
        cmd = input("").strip().lower()
        if cmd == 'r' and not is_recording:
            os.makedirs("ai_datasets", exist_ok=True)
            filename = f"ai_datasets/demo_{int(time.time())}.csv"
            record_file = open(filename, 'w', newline='')
            csv_writer = csv.writer(record_file)
            
            # Header: Time + L1~L7 + R1~R7 + Pan + Tilt
            header = ["timestamp"] + [f"L{i}" for i in range(1, 8)] + [f"R{i}" for i in range(1, 8)] + ["Pan", "Tilt"]
            csv_writer.writerow(header)
            
            record_start_time = time.time()
            is_recording = True
            print(f"\n[REC] Recording Started! Saving to {filename}...")
            
        elif cmd == 's' and is_recording:
            is_recording = False
            record_file.close()
            record_file = None
            csv_writer = None
            print("\n[STOP] Recording Stopped and File Saved.")

threading.Thread(target=recorder_command_thread, daemon=True).start()

# ==========================================
# 4. Hardware Initialization
# ==========================================
portHandler_left = PortHandler(PORT_LEFT)
packetHandler_left, style_left = build_packet_handler(portHandler_left)

portHandler_right = PortHandler(PORT_RIGHT)
packetHandler_right, style_right = build_packet_handler(portHandler_right)

if portHandler_left.openPort() and portHandler_left.setBaudRate(BAUDRATE): left_opened = True
if portHandler_right.openPort() and portHandler_right.setBaudRate(BAUDRATE): right_opened = True

if left_opened and packetHandler_left:
    for motor_id in MOTORS_LEFT:
        write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_TORQUE_ENABLE, 1)
        write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_ACCELERATION, 50)
        
if right_opened and packetHandler_right:
    for motor_id in MOTORS_RIGHT:
        write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_TORQUE_ENABLE, 1)
        write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_ACCELERATION, 50)
    
    write1(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_TORQUE_ENABLE, 1)
    write1(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_TORQUE_ENABLE, 1)
    write1(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_ACCELERATION, 50)
    write1(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_ACCELERATION, 100)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ==========================================
# 5. Main Control Loop
# ==========================================
try:
    while is_running:
        data, addr = sock.recvfrom(1024)
        latest_data = data

        sock.setblocking(False)
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                latest_data = data
            except Exception: break
        sock.setblocking(True)

        message = latest_data.decode('utf-8')

        if message.startswith("<") and message.endswith(">"):
            clean_msg = message.strip("<>")
            tick_strs = clean_msg.split(",")
            field_count = len(tick_strs)

            if field_count >= 21:
                try:
                    ticks = [int(val) for val in tick_strs[:20]]
                    
                    # 1. Physical AI Data Logging (NEW FEATURE)
                    if is_recording and csv_writer is not None:
                        elapsed = time.time() - record_start_time
                        log_data = [f"{elapsed:.4f}"] + ticks[:16] # Time + 16 Joints
                        csv_writer.writerow(log_data)
                    
                    # 2. Write Commands to Motors
                    for i in range(7):
                        write2(portHandler_left, packetHandler_left, style_left, MOTORS_LEFT[i], ADDR_GOAL_POSITION, ticks[i])
                    for i in range(7):
                        write2(portHandler_right, packetHandler_right, style_right, MOTORS_RIGHT[i], ADDR_GOAL_POSITION, ticks[i+7])

                    write2(portHandler_right, packetHandler_right, style_right, PAN_ID, ADDR_GOAL_POSITION, ticks[14])
                    write2(portHandler_right, packetHandler_right, style_right, TILT_ID, ADDR_GOAL_POSITION, ticks[15])

                    # 3. Lift & Wheel Control
                    lift_state = ticks[16]
                    update_lift_motor(lift_state)

                    if not is_auto_changing:
                        if current_lift_state == 1 and limit_top.is_pressed: stop_lift()
                        elif current_lift_state == -1 and limit_bottom.is_pressed: stop_lift()

                    dummy_parsed = [0] * 20
                    dummy_parsed[17] = ticks[18]
                    dummy_parsed[18] = ticks[19]
                    wheels_safe.update_wheels(dummy_parsed, ticks[17])

                except ValueError: pass


except KeyboardInterrupt:
    print("\nStopping system...")

finally:
    is_running = False
    if is_recording and record_file: record_file.close()
    try: wheels_safe.stop_all()
    except: pass
    if left_opened and packetHandler_left:
        for motor_id in MOTORS_LEFT: write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_TORQUE_ENABLE, 0)
        portHandler_left.closePort()
    if right_opened and packetHandler_right:
        for motor_id in MOTORS_RIGHT + [PAN_ID, TILT_ID]: write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_TORQUE_ENABLE, 0)
        portHandler_right.closePort()
    stop_lift()
    sock.close(); feedback_sock.close()
    print("Shutdown complete.")
