import socket
import struct
import os
import threading
import queue
import cv2
import numpy as np
import pygame
import collections
import time
from gtts import gTTS
import speech_recognition as sr
import customtkinter as ctk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ==========================================
# 1. Configuration & Globals
# ==========================================f
PI_IP = "192.168.0.16" 
VIDEO_PORT = 5007       
TTS_PORT = 5006         
CMD_PORT = 5008         
LOAD_PORT = 5009        
FEEDBACK_PORT = 5010    

pygame.mixer.init()
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

log_queue = queue.Queue()
latest_frame = None
frame_lock = threading.Lock()

load_left_buffer = collections.deque(np.zeros(50), maxlen=50)
load_right_buffer = collections.deque(np.zeros(50), maxlen=50)

# Hardware States
current_left_gripper = "빈손(Empty)"
current_right_gripper = "빈손(Empty)"
current_sw1 = "OFF"
current_sw2 = "OFF"
last_macro_ai_state = '0'

# ==========================================
# 2. Background Threads
# ==========================================
def tts_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", TTS_PORT))
    log_queue.put("[TTS] Server listening on port 5006...")
    
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            ai_text = data.decode('utf-8')
            log_queue.put(f"[AI 분석] {ai_text}")
            
            filename = f"alert_temp_{int(time.time())}.mp3"
            gTTS(text=ai_text, lang='ko').save(filename)
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
                
            pygame.mixer.music.unload()
            try: os.remove(filename)
            except: pass
        except Exception as e:
            log_queue.put(f"[TTS Error] 음성 출력 실패: {e}")

def voice_command_listener():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source: recognizer.adjust_for_ambient_noise(source)
    log_queue.put("[Voice] Mic ready. Say '분석' to trigger AI.")

    while True:
        try:
            with mic as source: audio = recognizer.listen(source, timeout=1, phrase_time_limit=3)
            text = recognizer.recognize_google(audio, language="ko-KR")
            if "분석" in text:
                log_queue.put(f"[Voice CMD] Keyword detected: '{text}'")
                cmd_sock.sendto(b"SPACE", (PI_IP, CMD_PORT))
                log_queue.put("[System] AI analysis request sent to Robot.")
        except sr.WaitTimeoutError: pass  
        except sr.UnknownValueError: pass  
        except Exception: pass

def video_receiver():
    global latest_frame
    while True:
        video_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            log_queue.put(f"[Video] Connecting to {PI_IP}:{VIDEO_PORT}...")
            video_sock.connect((PI_IP, VIDEO_PORT))
            log_queue.put("[Video] Stream connected successfully!")
            
            data = b""
            payload_size = struct.calcsize(">L")
            while True:
                while len(data) < payload_size: 
                    packet = video_sock.recv(4096)
                    if not packet: raise ConnectionError("Video stream lost.")
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack(">L", packed_msg_size)[0]
                
                while len(data) < msg_size: 
                    packet = video_sock.recv(4096)
                    if not packet: raise ConnectionError("Video stream lost.")
                    data += packet
                frame_data = data[:msg_size]
                data = data[msg_size:]

                frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                with frame_lock: latest_frame = frame.copy()
        except Exception as e:
            log_queue.put(f"[Video Error] {e}. Retrying in 3 seconds...")
            time.sleep(3)
        finally:
            video_sock.close()

def telemetry_receiver():
    global current_left_gripper, current_right_gripper, current_sw1, current_sw2, last_macro_ai_state
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOAD_PORT))
    log_queue.put(f"[Telemetry] Listening on port {LOAD_PORT}...")
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            parts = data.decode('utf-8').split(',')
            
            if len(parts) >= 3:
                load_left_buffer.append(int(parts[0]))
                load_right_buffer.append(int(parts[1]))
                
                keys_str = parts[2]
                
                # [수정] 7자리 키 패킷 파싱 로직 적용 (평상시가 0이므로, 눌렸을 때 '1'로 인식하도록 반전)
                if len(keys_str) >= 7:
                    if keys_str[4] == '1':    # 5번 키 (전체탈거)
                        current_left_gripper = "빈손(Empty)"
                        current_right_gripper = "빈손(Empty)"
                    else:
                        if keys_str[0] == '1': current_right_gripper = "미세(Fine)"
                        elif keys_str[1] == '1': current_right_gripper = "니퍼(Nipper)"
                        
                        if keys_str[2] == '1': current_left_gripper = "기본(Default)"
                        elif keys_str[3] == '1': current_left_gripper = "바이스(Vise)"

                    # 7번 키(AI 호출) 인덱스 6 파싱 (정상 반전)
                    current_ai_state = keys_str[6]
                    if current_ai_state == '1' and last_macro_ai_state == '0':
                        log_queue.put("[Macro CMD] 7번 키 입력 감지: AI 분석 요청 전송")
                        cmd_sock.sendto(b"SPACE", (PI_IP, CMD_PORT))
                    last_macro_ai_state = current_ai_state

            if len(parts) >= 4:
                current_sw1 = "ON" if parts[3] == '1' else "OFF"
            if len(parts) >= 5:
                current_sw2 = "ON" if parts[4] == '1' else "OFF"
        except Exception as e:
            pass # 패킷 깨짐 무시

def position_feedback_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", FEEDBACK_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            # 콘솔 전용 피드백이므로 패스
        except Exception:
            pass

threading.Thread(target=tts_server, daemon=True).start()
threading.Thread(target=voice_command_listener, daemon=True).start()
threading.Thread(target=video_receiver, daemon=True).start()
threading.Thread(target=telemetry_receiver, daemon=True).start()
threading.Thread(target=position_feedback_receiver, daemon=True).start()

# ==========================================
# 3. Main GUI Application
# ==========================================
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class SuwanDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Project Suwan - System Dashboard")
        self.geometry("1280x720")
        self.minsize(1024, 600)
        
        self.grid_columnconfigure(0, weight=6)
        self.grid_columnconfigure(1, weight=4)
        self.grid_rowconfigure(0, weight=1)

        # --- LEFT FRAME ---
        self.left_frame = ctk.CTkFrame(self)
        self.left_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.left_frame.grid_rowconfigure(1, weight=7)  
        self.left_frame.grid_rowconfigure(3, weight=3)  

        self.top_status_frame = ctk.CTkFrame(self.left_frame, fg_color="transparent")
        self.top_status_frame.grid(row=0, column=0, sticky="ew", pady=(10, 0), padx=10)
        self.top_status_frame.grid_columnconfigure(0, weight=1)
        self.top_status_frame.grid_columnconfigure(1, weight=1)

        self.lbl_title = ctk.CTkLabel(self.top_status_frame, text="Real-Time Vision & Thermal Monitor", font=ctk.CTkFont(size=20, weight="bold"))
        self.lbl_title.grid(row=0, column=0, sticky="w")

        self.lbl_sys_health = ctk.CTkLabel(self.top_status_frame, text="📡 Ping: OK  |  🔋 BAT: OK  |  🛑 E-STOP: OK", font=ctk.CTkFont(size=14, weight="bold"), text_color="#2ecc71")
        self.lbl_sys_health.grid(row=0, column=1, sticky="e")

        self.lbl_video = ctk.CTkLabel(self.left_frame, text="Waiting for Video Stream...")
        self.lbl_video.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        
        self.lbl_log_title = ctk.CTkLabel(self.left_frame, text="System & AI Log", font=ctk.CTkFont(size=16, weight="bold"))
        self.lbl_log_title.grid(row=2, column=0, pady=(10, 0), sticky="w", padx=10)
        
        self.textbox_log = ctk.CTkTextbox(self.left_frame, wrap="word", state="disabled", height=150)
        self.textbox_log.grid(row=3, column=0, padx=10, pady=(0, 10), sticky="nsew")

        # --- RIGHT FRAME ---
        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=0, column=1, padx=(0, 10), pady=10, sticky="nsew")
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(1, weight=1)  
        self.right_frame.grid_rowconfigure(3, weight=9)  

        self.lbl_status_title = ctk.CTkLabel(self.right_frame, text="Hardware Status", font=ctk.CTkFont(size=16, weight="bold"))
        self.lbl_status_title.grid(row=0, column=0, pady=(10, 5), sticky="w", padx=10)
        
        self.frame_status = ctk.CTkFrame(self.right_frame, fg_color="transparent")
        self.frame_status.grid(row=1, column=0, sticky="nsew", padx=10)
        
        self.lbl_gripper_left = ctk.CTkLabel(self.frame_status, text="Left Gripper: 빈손(Empty)")
        self.lbl_gripper_left.pack(anchor="w")
        self.lbl_gripper_right = ctk.CTkLabel(self.frame_status, text="Right Gripper: 빈손(Empty)")
        self.lbl_gripper_right.pack(anchor="w")
        self.lbl_sw1 = ctk.CTkLabel(self.frame_status, text="Wheel Control (SW1): OFF")
        self.lbl_sw1.pack(anchor="w", pady=(10, 0))
        self.lbl_sw2 = ctk.CTkLabel(self.frame_status, text="Pan/Tilt Control (SW2): OFF")
        self.lbl_sw2.pack(anchor="w")

        self.lbl_graph_title = ctk.CTkLabel(self.right_frame, text="Predictive Maintenance\n(Real-time Motor Load)", font=ctk.CTkFont(size=15, weight="bold"), justify="left")
        self.lbl_graph_title.grid(row=2, column=0, pady=(15, 5), sticky="w", padx=10)

        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(3, 4), dpi=100)
        self.fig.patch.set_facecolor('#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.set_ylim(0, 1000)  
        
        self.line_left, = self.ax.plot(list(range(50)), load_left_buffer, color='#3a7ebf', linewidth=2, label='Left Gripper')
        self.line_right, = self.ax.plot(list(range(50)), load_right_buffer, color='#e24a33', linewidth=2, label='Right Gripper')
        self.ax.legend(loc='upper right', facecolor='#2b2b2b', edgecolor='none', labelcolor='white')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=3, column=0, padx=10, pady=(0, 10), sticky="nsew")

        self.update_gui()

    def update_gui(self):
        global latest_frame, current_left_gripper, current_right_gripper, current_sw1, current_sw2
        with frame_lock:
            frame_copy = latest_frame.copy() if latest_frame is not None else None

        if frame_copy is not None:
            guide_x, guide_y = 640, 240
            cv2.putText(frame_copy, "[ Controller Guide ]", (guide_x + 15, guide_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            guides = [
                "Key 1: Right Gripper (Fine)",
                "Key 2: Right Gripper (Nipper)",
                "Key 3: Left Gripper (Default)",
                "Key 4: Left Gripper (Vise)",
                "Key 5: Both Arms (Detach)",
                "Key 7: AI Situation Analysis"
            ]
            
            for i, text in enumerate(guides):
                color = (0, 255, 255) if i == 5 else (200, 200, 200)
                cv2.putText(frame_copy, text, (guide_x + 15, guide_y + 75 + (i * 28)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

            frame_rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            
            # [수정] 프로그램 시작 시 UI 렌더링 지연으로 크기가 0이 되어 다운되는 현상 방지
            target_width = max(640, self.left_frame.winfo_width() - 40)
            target_height = max(480, self.left_frame.winfo_height() - self.textbox_log.winfo_reqheight() - 80)
            
            if target_width > 10 and target_height > 10:
                img.thumbnail((target_width, target_height), Image.Resampling.LANCZOS)
                ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(img.width, img.height))
                self.lbl_video.configure(image=ctk_img, text="")

        self.lbl_gripper_left.configure(text=f"Left Gripper: {current_left_gripper}")
        self.lbl_gripper_right.configure(text=f"Right Gripper: {current_right_gripper}")
        self.lbl_sw1.configure(text=f"Wheel Control (SW1): {current_sw1}")
        self.lbl_sw2.configure(text=f"Pan/Tilt Control (SW2): {current_sw2}")

        while not log_queue.empty():
            msg = log_queue.get()
            self.textbox_log.configure(state="normal")
            self.textbox_log.insert("end", msg + "\n")
            self.textbox_log.see("end")  
            self.textbox_log.configure(state="disabled")

        self.line_left.set_ydata(load_left_buffer)
        self.line_right.set_ydata(load_right_buffer)
        self.canvas.draw_idle()

        self.after(30, self.update_gui)

if __name__ == "__main__":
    app = SuwanDashboard()
    app.mainloop()