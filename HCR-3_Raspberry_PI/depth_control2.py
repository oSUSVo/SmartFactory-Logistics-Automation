# depth_control.py
import pyrealsense2 as rs
import numpy as np
import serial
import cv2
import time
import os
import sys
import threading

# -------------------------------
# ⚙️ 설정
# -------------------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

box_size = 30  
A_center_x = 315  
A_center_y = 250  

start_x = A_center_x - (box_size // 2)
start_y = A_center_y - (box_size // 2)

roi_A = (start_x, start_y, box_size, box_size)
roi_B = (start_x - box_size, start_y, box_size, box_size)
roi_C = (start_x - (box_size // 2), start_y, box_size, box_size)

# =====================================================================
# 브릿지(hcr3_bridge) 명령 수신 쓰레드
# =====================================================================
check_requested = False

def listen_to_bridge():
    global check_requested, ir_start_time
    for line in sys.stdin:
        cmd = line.strip()
        
        # 1. 서버(Bridge)로부터 시작 신호 수신 시 컨베이어 가동
        if cmd == "START":
            print("🚀 서버 시작 신호 수신: 컨베이어 벨트를 가동합니다.", flush=True)
            if ser: ser.write(b'S') # STM32에 컨베이어 가동('S') 송신

        # 2. Storage 로봇이 물건을 놓았을 때 판정 시작 명령
        elif cmd == "CHECK_IR":
            print("📩 적외선 4초 판정 시작.", flush=True)
            check_requested = True

        elif cmd == "STOP_CHECK":
            check_requested = False
            ir_start_time = None

# =====================================================================

def get_distance(depth_array, roi):
    x, y, w, h = roi
    roi_depth = depth_array[y:y+h, x:x+w] * depth_scale
    valid = roi_depth[(roi_depth > 0.2) & (roi_depth < 1.5)]
    if valid.size < 10: return 9.99
    return np.median(valid)

# 시리얼 및 카메라 초기화
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ STM32 연결 성공")
except:
    ser = None
    print("⚠️ STM32 연결 실패 (테스트 모드)")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

print("카메라 켜는 중...")
profile = pipeline.start(config)
time.sleep(2)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
colorizer = rs.colorizer()

print("📍 바닥측정(3초 대기)")
time.sleep(3)

samples_A, samples_B, samples_C = [], [], []
for _ in range(30):
    frames = pipeline.wait_for_frames()
    depth_array = np.asanyarray(frames.get_depth_frame().get_data())
    samples_A.append(get_distance(depth_array, roi_A))
    samples_B.append(get_distance(depth_array, roi_B))
    samples_C.append(get_distance(depth_array, roi_C))

BASE_A, BASE_B, BASE_C = np.mean(samples_A), np.mean(samples_B), np.mean(samples_C)
print(f"✅ 측정 완료 | A바닥: {BASE_A:.3f}m | B바닥: {BASE_B:.3f}m | C바닥: {BASE_C:.3f}m\n")

# 상태 제어 변수
waiting_for_item = None  
is_completed = False     
empty_counter = 0        
system_halted = False    

# 적외선 판정 변수
ir_detected = False
ir_start_time = None

if ser:
    ser.write(b'A'); ser.write(b'B'); ser.write(b'C')

# 2. 시리얼(Hardware)의 센서값을 듣는 함수
def listen_to_serial():
    global ir_detected
    while True:
        if ser:
            try:
                incoming = ser.readline().decode('utf-8').strip()
                if incoming == 'H': ir_detected = True
                elif incoming == 'L': ir_detected = False
            except: pass

# 3. 두 스레드를 각각 실행
threading.Thread(target=listen_to_bridge, daemon=True).start()
threading.Thread(target=listen_to_serial, daemon=True).start()

print("🎥 모니터링 시작")

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue
        
        depth_array = np.asanyarray(depth_frame.get_data())
        d_A, d_B, d_C = get_distance(depth_array, roi_A), get_distance(depth_array, roi_B), get_distance(depth_array, roi_C)
        h_A, h_B, h_C = BASE_A - d_A, BASE_B - d_B, BASE_C - d_C

        has_A = 0.025 < h_A < 0.115
        has_B = 0.025 < h_B < 0.115
        has_C = 0.070 < h_C < 0.115
        is_interfering = (h_A > 0.12) or (h_B > 0.12) or (h_C > 0.12)

        if not system_halted:
            # 0. 우선순위 가로채기
            if waiting_for_item == 'b' and not has_A:
                waiting_for_item = None; empty_counter = 0
            elif waiting_for_item == 'c' and (not has_A or not has_B):
                waiting_for_item = None; empty_counter = 0

            # 1. 빈자리 찾기 및 릴레이 제어 (A > B > C)
            if waiting_for_item is None and not is_interfering:
                if has_A and has_B and has_C:
                    empty_counter = 0 
                    if not is_completed:
                        print("🎉 모든 칸 완료!"); sys.stdout.flush()
                        if ser: ser.write(b'A'); ser.write(b'B'); ser.write(b'C')
                        is_completed = True
                else:
                    is_completed = False
                    empty_counter += 1 
                    if empty_counter > 15:
                        if not has_A:
                            print("[REQUEST_BOX] 1"); sys.stdout.flush()
                            waiting_for_item = 'a'
                            if ser: ser.write(b'B'); ser.write(b'C'); ser.write(b'a')
                        elif not has_B:
                            print("[REQUEST_BOX] 1"); sys.stdout.flush()
                            waiting_for_item = 'b'
                            if ser: ser.write(b'A'); ser.write(b'C'); ser.write(b'b')
                        elif not has_C:
                            print("[REQUEST_BOX] 1"); sys.stdout.flush()
                            waiting_for_item = 'c'
                            if ser: ser.write(b'A'); ser.write(b'B'); ser.write(b'c')
                        empty_counter = 0 

            # 3. 뎁스 카메라 안착 확인 (릴레이 OFF)
            if waiting_for_item == 'a' and has_A:
                if ser: ser.write(b'A'); waiting_for_item = None
            elif waiting_for_item == 'b' and has_B:
                if ser: ser.write(b'B'); waiting_for_item = None
            elif waiting_for_item == 'c' and has_C:
                if ser: ser.write(b'C'); waiting_for_item = None

            # 4. 적외선 4초 판정 (요청 시에만 작동)
            if check_requested:
                if ir_detected:
                    print("[IR_STATE] 1", flush=True)
                else:
                    print("[IR_STATE] 0", flush=True)

        # -------------------------------
        # 화면 표시 (디버깅용 UI)
        # -------------------------------
        depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        for (x, y, w, h), h_val, has_block, label in zip(
            [roi_B, roi_C, roi_A],
            [h_B, h_C, h_A],
            [has_B, has_C, has_A],
            ["B(L)", "C(M)", "A(R)"]
        ):
            color = (0, 0, 255) if has_block else (0, 255, 0)
            cv2.rectangle(depth_color, (x, y), (x+w, y+h), color, 2)
            cv2.putText(depth_color, f"{label}:{h_val*100:.1f}cm", (x-10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 상태 텍스트 출력
        if is_interfering:
            ui_state = "WARNING: Robot Detected! Paused."
            ui_color = (0, 0, 255)
        elif system_halted:
            ui_state = "STATE: SYSTEM HALTED"
            ui_color = (0, 0, 255)
        elif waiting_for_item is not None:
            ui_state = f"STATE: WAITING FOR {waiting_for_item.upper()}..."
            ui_color = (0, 255, 255)
        elif has_A and has_B and has_C:
            ui_state = "ALL DONE! PERFECT."
            ui_color = (0, 255, 0)
        else:
            ui_state = "STATE: CHECKING..."
            ui_color = (255, 255, 255)

        cv2.putText(depth_color, ui_state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ui_color, 2)
        cv2.imshow("Depth Check", depth_color)

        cv2.imshow("Depth Check", depth_color)
        print("imshow 호출됨", flush=True)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser: ser.close()