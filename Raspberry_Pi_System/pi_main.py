import cv2
import numpy as np
import time
import threading
import socket
import struct
import board
import busio
import adafruit_mlx90640
from picamera2 import Picamera2
import PIL.Image
from google import genai 

# ==========================================
# 1. Configuration & API Key
# ==========================================
GEMINI_API_KEY = "YOUR_GEMINI_API_KEY_HERE"
try:
    client = genai.Client(api_key=GEMINI_API_KEY)
except Exception as e:
    print(f"GenAI Init Error: {e}")

I2C_BUS = board.I2C()
MLX_I2C_ADDR = 0x33
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
WARN_TEMP_THRESHOLD = 40.0

LAPTOP_IP = "192.168.0.3"
LAPTOP_PORT = 5006  
VIDEO_PORT = 5007   
CMD_PORT = 5008     

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# 2. Shared Globals
# ==========================================
thermal_img_global = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
max_temp_global = 0.0
bbox_rect_global = None 
is_running = True
frame_lock = threading.Lock()

last_alert_time = 0.0
ALERT_COOLDOWN = 10.0
ai_analyzing = False
trigger_ai = False  

# ==========================================
# 3. AI Analysis & Thermal Threads
# ==========================================
def run_ai_analysis(frame_bgr, current_temp):
    global ai_analyzing
    print("[AI] Requesting situation analysis...")
    try:
        cv2.imwrite("temp_ai_scene.jpg", frame_bgr)
        img = PIL.Image.open("temp_ai_scene.jpg")

        prompt = f"""
        You are a safety assistant robot in a waste battery recycling plant.
        The operator just requested a situation analysis. The max temperature is {current_temp} C.
        Look at the attached photo, identify the current scene, and brief the operator.
        IMPORTANT: Your response MUST be exactly 1 sentence, spoken in polite, professional Korean.
        """
        response = client.models.generate_content(
            model='gemini-3.6-flash',
            contents=[prompt, img]
        )
        ai_text = response.text.replace('*', '').strip()
        print(f"[AI Result]: {ai_text}")

        udp_sock.sendto(ai_text.encode('utf-8'), (LAPTOP_IP, LAPTOP_PORT))
        print("[AI] Transmission complete.")
    except Exception as e:
        print(f"[AI Error] {e}")
    finally:
        ai_analyzing = False

def update_thermal():
    global thermal_img_global, is_running, max_temp_global, bbox_rect_global
    try:
        mlx = adafruit_mlx90640.MLX90640(I2C_BUS, address=MLX_I2C_ADDR)
        mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
    except Exception as e:
        print(f"MLX Init Error: {e}")
        is_running = False
        return

    frame = np.zeros((24 * 32,))
    while is_running:
        try:
            mlx.getFrame(frame)
            data_array = np.reshape(frame, (24, 32))
            max_temp = np.max(data_array)
            max_idx = np.argmax(data_array)
            max_y, max_x = divmod(max_idx, 32)

            scale_x = FRAME_WIDTH / 32.0
            scale_y = FRAME_HEIGHT / 24.0
            center_x = int((max_x + 0.5) * scale_x)
            center_y = int((max_y + 0.5) * scale_y)

            box_size = 80
            box_x = max(0, center_x - box_size // 2)
            box_y = max(0, center_y - box_size // 2)

            if box_x + box_size > FRAME_WIDTH: box_x = FRAME_WIDTH - box_size
            if box_y + box_size > FRAME_HEIGHT: box_y = FRAME_HEIGHT - box_size

            real_x = FRAME_WIDTH - box_x - box_size 

            norm_img = cv2.normalize(data_array, None, 0, 255, cv2.NORM_MINMAX)
            norm_img = np.uint8(norm_img)
            color_img = cv2.applyColorMap(norm_img, cv2.COLORMAP_JET)
            resized_img = cv2.resize(color_img, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_CUBIC)

            with frame_lock:
                thermal_img_global = cv2.flip(resized_img, 1)
                max_temp_global = max_temp
                if max_temp >= WARN_TEMP_THRESHOLD:
                    bbox_rect_global = (real_x, box_y, box_size, box_size)
                else:
                    bbox_rect_global = None
        except ValueError:
            continue
        except Exception as e:
            time.sleep(0.1)

def command_listener():
    global trigger_ai, is_running
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(("0.0.0.0", CMD_PORT))
    while is_running:
        data, _ = cmd_sock.recvfrom(1024)
        if data.decode('utf-8') == "SPACE":
            trigger_ai = True

# ==========================================
# 4. Main Loop (Capture & TCP Send)
# ==========================================
def main():
    global is_running, last_alert_time, ai_analyzing, trigger_ai
    
    print("Initializing PiCamera2...")
    picam2 = Picamera2()
    try:
        config = picam2.create_video_configuration(main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)})
        picam2.configure(config)
        picam2.start()
    except Exception as e:
        print(f"PiCam Init Error: {e}")
        return

    print("Starting Thermal Thread...")
    t_thread = threading.Thread(target=update_thermal)
    t_thread.daemon = True
    t_thread.start()
    
    print("Starting Command Listener Thread...")
    c_thread = threading.Thread(target=command_listener)
    c_thread.daemon = True
    c_thread.start()
    
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind(("0.0.0.0", VIDEO_PORT))
    server_sock.listen(1)
    print(f"Waiting for laptop to connect on port {VIDEO_PORT}...")
    
    conn, addr = server_sock.accept()
    print(f"Connected by {addr}! Starting Stream...")

    try:
        while is_running:
            v_frame = picam2.capture_array("main")

            with frame_lock:
                current_thermal = thermal_img_global.copy()
                current_max_temp = max_temp_global
                current_bbox = bbox_rect_global

            # Auto Alert & BBOX Drawing
            if current_bbox is not None:
                x, y, w, h = current_bbox
                curr_time = time.time()
                if (curr_time - last_alert_time) > ALERT_COOLDOWN:
                    alert_msg = "\uc804\ubc29\uc5d0 \uace0\uc628\uc758 \ubc1c\uc5f4\ubb3c\uc9c8\uc774 \uac10\uc9c0\ub418\uc5c8\uc2b5\ub2c8\ub2e4."
                    try:
                        udp_sock.sendto(alert_msg.encode('utf-8'), (LAPTOP_IP, LAPTOP_PORT))
                    except Exception: pass
                    last_alert_time = curr_time

                color = (0, 0, 255)
                label = f"WARN: {current_max_temp:.1f} C"
                X_OFFSET = 0
                v_x = min(x + X_OFFSET, FRAME_WIDTH - w)

                cv2.rectangle(v_frame, (v_x, y), (v_x+w, y+h), color, 3)
                cv2.putText(v_frame, label, (v_x, max(20, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.rectangle(current_thermal, (x, y), (x+w, y+h), color, 3)
                cv2.putText(current_thermal, label, (x, max(20, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            else:
                color = (255, 255, 255)
                label = f"TEMP: {current_max_temp:.1f} C"
                cv2.putText(v_frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(current_thermal, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if ai_analyzing:
                cv2.putText(v_frame, "AI ANALYZING...", (FRAME_WIDTH - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            small_thermal = cv2.resize(current_thermal, (320, 240), interpolation=cv2.INTER_LINEAR)
            padded_thermal = np.zeros((FRAME_HEIGHT, 320, 3), dtype=np.uint8)
            padded_thermal[0:240, 0:320] = small_thermal  
            combined_frame = np.hstack((v_frame, padded_thermal))

            _, buffer = cv2.imencode('.jpg', combined_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            msg = struct.pack(">L", len(buffer)) + buffer.tobytes()
            conn.sendall(msg)

            if trigger_ai:
                trigger_ai = False
                if not ai_analyzing:
                    ai_analyzing = True
                    ai_thread = threading.Thread(target=run_ai_analysis, args=(v_frame.copy(), current_max_temp))
                    ai_thread.daemon = True
                    ai_thread.start()

    except Exception as e:
        print(f"Stream Error: {e}")
    finally:
        is_running = False
        conn.close()
        server_sock.close()
        picam2.stop()
        udp_sock.close()
        print("System shutdown complete.")

if __name__ == "__main__":
    main()
