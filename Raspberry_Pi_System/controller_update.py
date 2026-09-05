import socket
import time
import threading
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
UDP_PORT = 5009      # [CRITICAL FIX] PC sends data to 5009, NOT 5007
TCP_PORT = 5006  

LAPTOP_IP = "192.168.0.3"  
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

# --- Lift & Limit Switch Setup ---
LIFT_PIN_1 = 17  
LIFT_PIN_2 = 27  
LIMIT_TOP_PIN = 23     
LIMIT_BOTTOM_PIN = 24  

lift_dir_1 = OutputDevice(LIFT_PIN_1, initial_value=False)
lift_dir_2 = OutputDevice(LIFT_PIN_2, initial_value=False)
limit_top = Button(LIMIT_TOP_PIN, pull_up=True)
limit_bottom = Button(LIMIT_BOTTOM_PIN, pull_up=True)

current_lift_state = 0  
is_auto_changing = False  

def stop_lift():
    global current_lift_state
    lift_dir_1.off()
    lift_dir_2.off()
    current_lift_state = 0

def update_lift_motor(new_state):
    global current_lift_state
    if is_auto_changing or new_state == current_lift_state:
        return
    lift_dir_1.off()
    lift_dir_2.off()
    time.sleep(0.05)
    if new_state == 1 and not limit_top.is_pressed:
        lift_dir_1.on()
        current_lift_state = 1
    elif new_state == -1 and not limit_bottom.is_pressed:
        lift_dir_2.on()
        current_lift_state = -1
    else:
        current_lift_state = 0

def tcp_server_thread():
    global is_auto_changing
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((UDP_IP, TCP_PORT))
    server_socket.listen(1)
    print(f"[TCP Server] Listening on port {TCP_PORT}...")
    while is_running:
        try:
            conn, addr = server_socket.accept()
            with conn:
                while True:
                    data = conn.recv(1024)
                    if not data: break
                    command = data.decode("utf-8").strip()
                    is_auto_changing = True  
                    if command == "DESCEND":
                        start_t = time.time()
                        stop_lift()
                        time.sleep(0.05)
                        lift_dir_2.on()
                        while not limit_bottom.is_pressed: time.sleep(0.01)
                        stop_lift()
                        elapsed = time.time() - start_t
                        conn.sendall(f"OK:{elapsed:.3f}".encode("utf-8"))
                    elif command.startswith("ASCEND:"):
                        seconds = float(command.split(":", 1)[1])
                        stop_lift()
                        time.sleep(0.05)
                        lift_dir_1.on()
                        start_t = time.time()
                        while (time.time() - start_t) < seconds:
                            if limit_top.is_pressed: break
                            time.sleep(0.01)
                        stop_lift()
                        conn.sendall("OK".encode("utf-8"))
                    elif command.startswith("CLEARANCE:"):
                        seconds = float(command.split(":", 1)[1])
                        stop_lift()
                        time.sleep(0.05)
                        lift_dir_1.on()
                        start_t = time.time()
                        while (time.time() - start_t) < seconds:
                            if limit_top.is_pressed: break
                            time.sleep(0.01)
                        stop_lift()
                        conn.sendall("OK".encode("utf-8"))
                    is_auto_changing = False 
        except Exception:
            is_auto_changing = False


# ==========================================
# 3. Motor Functions
# ==========================================
def scs_write_pos(ph, pkt, style, servo_id, pos):
    pos_val = int(pos)
    write1(ph, pkt, style, servo_id, 42, (pos_val >> 8) & 0xFF)
    write1(ph, pkt, style, servo_id, 43, pos_val & 0xFF)

def get_motor_load(ph, pkt, style, motor_id):
    raw_load, result, error = read2(ph, pkt, style, motor_id, ADDR_PRESENT_LOAD)
    return raw_load & 0x3FF if result == 0 else 0

def telemetry_sender_thread():
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while is_running:
        if left_opened and right_opened:
            load_L = get_motor_load(portHandler_left, packetHandler_left, style_left, MOTORS_LEFT[6])
            load_R = get_motor_load(portHandler_right, packetHandler_right, style_right, MOTORS_RIGHT[6])
            
            msg = f"{load_L},{load_R},{latest_keys_str},{latest_sw1},{latest_sw2}"
            try: 
                telemetry_sock.sendto(msg.encode('utf-8'), (LAPTOP_IP, LOAD_PORT))
            except Exception: 
                pass
        time.sleep(0.05)
    telemetry_sock.close()


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

print("Robot System Ready. Listening on UDP", UDP_PORT)

threading.Thread(target=tcp_server_thread, daemon=True).start()
threading.Thread(target=telemetry_sender_thread, daemon=True).start()

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
            except Exception:
                break
        sock.setblocking(True)

        message = latest_data.decode('utf-8')

        if message.startswith("<") and message.endswith(">"):
            clean_msg = message.strip("<>")
            tick_strs = clean_msg.split(",")
            
            field_count = len(tick_strs)

            # [FIX] Handle both Normal(22) and Shutdown(21) packets safely
            if field_count >= 21:
                try:
                    # Parse Integer fields (0 to 19)
                    ticks = [int(val) for val in tick_strs[:20]]
                    
                    # Parse String field (20) - Keep as string to preserve leading zeros
                    latest_keys_str = str(tick_strs[20])
                    
                    # Parse SW2 (21) if it exists (Normal packet)
                    if field_count >= 22:
                        latest_sw2 = int(tick_strs[21])
                    else:
                        latest_sw2 = 0 # Shutdown packet fallback

                    latest_sw1 = ticks[17]
                    throttle_val = ticks[18]
                    turn_val = ticks[19]

                    # 1. Write Commands 
                    for i in range(7):
                        write2(portHandler_left, packetHandler_left, style_left, MOTORS_LEFT[i], ADDR_GOAL_POSITION, ticks[i])
                    for i in range(7):
                        write2(portHandler_right, packetHandler_right, style_right, MOTORS_RIGHT[i], ADDR_GOAL_POSITION, ticks[i+7])

                    scs_write_pos(portHandler_right, packetHandler_right, style_right, PAN_ID, ticks[14])
                    scs_write_pos(portHandler_right, packetHandler_right, style_right, TILT_ID, ticks[15])

                    # 2. Read Feedback 
                    present_left = []
                    for motor_id in MOTORS_LEFT:
                        pos, res, err = read2(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_PRESENT_POSITION)
                        present_left.append(pos if res == 0 else "NA")

                    present_right = []
                    for motor_id in MOTORS_RIGHT:
                        pos, res, err = read2(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_PRESENT_POSITION)
                        present_right.append(pos if res == 0 else "NA")

                    # Send feedback packet to controller laptop
                    feedback_str = "<" + ",".join(map(str, present_left)) + "," + ",".join(map(str, present_right)) + ">"
                    try:
                        feedback_sock.sendto(feedback_str.encode('utf-8'), (addr[0], FEEDBACK_PORT))
                    except Exception:
                        pass

                    # --- Lift & Wheel Control ---
                    lift_state = ticks[16]
                    update_lift_motor(lift_state)

                    if not is_auto_changing:
                        if current_lift_state == 1 and limit_top.is_pressed:
                            stop_lift()
                        elif current_lift_state == -1 and limit_bottom.is_pressed:
                            stop_lift()

                    # Wheel control mapping
                    dummy_parsed = [0] * 20
                    dummy_parsed[17] = throttle_val
                    dummy_parsed[18] = turn_val
                    wheels_safe.update_wheels(dummy_parsed, latest_sw1)

                except ValueError as e:
                    print(f"[WARN] Failed to parse packet, skipping frame. Err: {e}")

except KeyboardInterrupt:
    print("\nStopping system...")
except Exception as e:
    print(f"\nException: {e}")

finally:
    is_running = False
    try: wheels_safe.stop_all()
    except: pass
    
    if left_opened and packetHandler_left:
        for motor_id in MOTORS_LEFT: 
            write1(portHandler_left, packetHandler_left, style_left, motor_id, ADDR_TORQUE_ENABLE, 0)
        portHandler_left.closePort()
        
    if right_opened and packetHandler_right:
        for motor_id in MOTORS_RIGHT + [PAN_ID, TILT_ID]: 
            write1(portHandler_right, packetHandler_right, style_right, motor_id, ADDR_TORQUE_ENABLE, 0)
        portHandler_right.closePort()
        
    stop_lift()
    limit_top.close(); limit_bottom.close(); lift_dir_1.close(); lift_dir_2.close()
    sock.close(); feedback_sock.close()
    print("Shutdown complete.")
