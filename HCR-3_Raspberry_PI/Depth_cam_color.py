
import pyrealsense2 as rs
import numpy as np
import serial
import cv2
import time

# -------------------------------
# ⚙️ 설정
# -------------------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# 로봇 동작 시간 (실제 동작에 맞춰 조절)
ROBOT_WORK_TIME = 4.0
STABILIZE_TIME = 2.0

# -------------------------------
# 🎯 ROI 영역 (화면 중앙 기준, 다닥다닥 붙어있는 상태)
# -------------------------------
roi_A = (290, 240, 30, 30)
roi_B = (320, 240, 30, 30)
roi_C = (305, 240, 30, 30)

# -------------------------------
# 시리얼 연결
# -------------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ STM32 연결 성공")
except:
    ser = None
    print("⚠️ STM32 연결 실패 (테스트 모드로 진행)")

# -------------------------------
# RealSense 초기화
# -------------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

print("카메라 켜는 중...")
profile = pipeline.start(config)
time.sleep(2)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# 🎨 [추가됨] 거리에 따라 자동으로 예쁜 색상을 입혀주는 컬러라이저 초기화
colorizer = rs.colorizer()

# -------------------------------
# 🌟 [수정됨] 거리 계산 함수 (Median 적용)
# -------------------------------
def get_distance(depth_array, roi):
    x, y, w, h = roi
    roi_depth = depth_array[y:y+h, x:x+w] * depth_scale

    # 0.2m ~ 1.5m 사이의 의미 있는 데이터만 추출
    valid = roi_depth[(roi_depth > 0.2) & (roi_depth < 1.5)]

    # 박스 안에 유효한 데이터가 거의 없으면 무시
    if valid.size < 10:
        return 9.99

    # [핵심] 일부 모서리가 걸치는 노이즈를 무시하기 위해 '중앙값(Median)' 사용
    # 박스의 50% 이상이 블록으로 채워져야만 정상 높이로 인정합니다.
    return np.median(valid)

# -------------------------------
# 1. 바닥 캘리브레이션
# -------------------------------
print("\n🚨 중요: 바닥을 측정합니다. 블록을 모두 치워주세요! (3초 대기)")
time.sleep(3)

samples_A, samples_B, samples_C = [], [], []

print("바닥 깊이 기록 중...")
for _ in range(30):
    frames = pipeline.wait_for_frames()
    depth_array = np.asanyarray(frames.get_depth_frame().get_data())
    samples_A.append(get_distance(depth_array, roi_A))
    samples_B.append(get_distance(depth_array, roi_B))
    samples_C.append(get_distance(depth_array, roi_C))

BASE_A = np.mean(samples_A)
BASE_B = np.mean(samples_B)
BASE_C = np.mean(samples_C)

print(f"✅ 측정 완료! | A바닥: {BASE_A:.3f}m | B바닥: {BASE_B:.3f}m | C바닥: {BASE_C:.3f}m\n")

# -------------------------------
# 상태 머신 제어 변수
# -------------------------------
state = "CHECK_A"
robot_busy = False
action_start_time = 0

last_has_A = False
last_has_B = False
last_has_C = False

print("🚀 자동화 시스템 시작 (q 누르면 종료)")

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue
        depth_array = np.asanyarray(depth_frame.get_data())

        d_A = get_distance(depth_array, roi_A)
        d_B = get_distance(depth_array, roi_B)
        d_C = get_distance(depth_array, roi_C)

        # ----------------------------------------------------
        # 🌟 [수정됨] 블록 높이 판단 임계값 (매우 중요)
        # 블록 1개의 높이는 약 4.5cm.
        # A, B(1층)는 바닥에서 3.0cm 이상 솟으면 인정.
        # C(2층)는 A,B 위에 쌓이므로 9.0cm가 됩니다. 따라서 7.5cm 이상 솟아야 인정.
        # ----------------------------------------------------
        has_A = (BASE_A - d_A) > 0.030
        has_B = (BASE_B - d_B) > 0.030
        has_C = (BASE_C - d_C) > 0.075

        current_time = time.time()

        # 블록 치워짐 감지 (STM32에 대문자 전송 -> 릴레이 OFF)
        if not robot_busy:
            if last_has_A and not has_A:
                print("⚠️ A 블록 치워짐 감지! -> 릴레이 OFF(A)")
                if ser: ser.write(b'A')
            if last_has_B and not has_B:
                print("⚠️ B 블록 치워짐 감지! -> 릴레이 OFF(B)")
                if ser: ser.write(b'B')
            if last_has_C and not has_C:
                print("⚠️ C 블록 치워짐 감지! -> 릴레이 OFF(C)")
                if ser: ser.write(b'C')

        last_has_A = has_A
        last_has_B = has_B
        last_has_C = has_C

        # --- 엄격한 상태 머신 로직 ---
        if robot_busy:
            if current_time - action_start_time > (ROBOT_WORK_TIME + STABILIZE_TIME):
                print("🔄 로봇팔 동작 완료 -> 다시 감지 시작")
                robot_busy = False
        else:
            if state == "CHECK_A":
                if not has_A:
                    print("⬇️ [명령] A 블록 없음 -> 릴레이 'a' 전송 -> 로봇 대기")
                    if ser: ser.write(b'a')
                    robot_busy = True
                    action_start_time = current_time
                else:
                    print("✅ A 블록 확인 완료 -> B 확인으로 넘어감")
                    state = "CHECK_B"

            elif state == "CHECK_B":
                if not has_B:
                    print("⬇️ [명령] B 블록 없음 -> 릴레이 'b' 전송 -> 로봇 대기")
                    if ser: ser.write(b'b')
                    robot_busy = True
                    action_start_time = current_time
                else:
                    print("✅ B 블록 확인 완료 -> C 확인으로 넘어감")
                    state = "CHECK_C"

            elif state == "CHECK_C":
                if has_A and has_B:
                    if not has_C:
                        print("⬇️ [명령] C 블록 없음 -> 릴레이 'c' 전송 -> 로봇 대기")
                        if ser: ser.write(b'c')
                        robot_busy = True
                        action_start_time = current_time
                    else:
                        if state != "DONE":
                            print("🎉 모든 블록 조립 완료! (블록을 치우면 다시 시작합니다)")
                            state = "DONE"
                else:
                    print("⚠️ A 또는 B 블록이 이탈함! 다시 A부터 검사합니다.")
                    state = "CHECK_A"

            elif state == "DONE":
                if not has_A or not has_B or not has_C:
                    print("🚨 블록 붕괴 감지! 처음부터 다시 조립을 시작합니다.")
                    state = "CHECK_A"


        # -------------------------------
        # 화면 표시 (디버깅용 UI)
        # -------------------------------
        # 🎨 [수정됨] 기존 파란색 고정 변환 방식을 지우고 RealSense 컬러라이저 적용
        colorized_depth = colorizer.colorize(depth_frame)
        depth_color = np.asanyarray(colorized_depth.get_data())

        for (x, y, w, h), d, base, has_block, label in zip(
            [roi_A, roi_C, roi_B],
            [d_A, d_C, d_B],
            [BASE_A, BASE_C, BASE_B],
            [has_A, has_C, has_B],
            ["A", "C", "B"]
        ):
            color = (0, 0, 255) if has_block else (0, 255, 0)
            cv2.rectangle(depth_color, (x, y), (x+w, y+h), color, 2)

            # 카메라 화면에 실제 높이(cm)가 얼마나 솟아올랐는지 실시간 표시
            height_diff = base - d
            cv2.putText(depth_color, f"{label} H:{height_diff*100:.1f}cm", (x-10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        ui_state = f"STATE: {state}" if not robot_busy else "STATE: ROBOT WORKING..."
        ui_color = (0, 255, 255) if not robot_busy else (0, 0, 255)
        cv2.putText(depth_color, ui_state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ui_color, 2)

        cv2.imshow("Depth Check", depth_color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if ser: ser.close()
