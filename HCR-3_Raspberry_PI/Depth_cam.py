import pyrealsense2 as rs
import numpy as np
import serial
import cv2

# -------------------------------
# ⚙️ 설정
# -------------------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# 블록 높이 (4.5cm)
BLOCK_HEIGHT = 0.045

# 바닥 기준 거리 (실행 후 로그 보고 수정 가능)
FLOOR_DISTANCE = 0.60

# 임계값
FLOOR_1_THRESH = FLOOR_DISTANCE - BLOCK_HEIGHT/2
FLOOR_2_THRESH = FLOOR_DISTANCE - BLOCK_HEIGHT*1.5

# -------------------------------
# 🎯 ROI 영역
# -------------------------------
roi1 = (120, 160, 140, 160)  # A 좌
roi3 = (250, 160, 140, 160)  # C 중앙
roi2 = (380, 160, 140, 160)  # B 우

# -------------------------------
# 시리얼 연결
# -------------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ STM32 연결 성공")
except:
    ser = None
    print("⚠️ STM32 연결 실패")

# -------------------------------
# RealSense 초기화
# -------------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# -------------------------------
# 상태 저장
# -------------------------------
last_s1, last_s2, last_s3 = ' ', ' ', ' '

# Depth 안정화 필터
FILTER_SIZE = 5
hist1 = []
hist2 = []
hist3 = []

# -------------------------------
# ROI 거리 계산
# -------------------------------
def get_min_distance(depth_array, roi):

    x, y, w, h = roi

    roi_depth = depth_array[y:y+h, x:x+w] * depth_scale

    valid = roi_depth[(roi_depth > 0.2) & (roi_depth < 1.5)]

    if valid.size < 30:
        return 9.99

    sorted_depth = np.sort(valid)

    sample = sorted_depth[:max(10, int(len(sorted_depth)*0.05))]

    return np.mean(sample)


# -------------------------------
# 실행
# -------------------------------
try:

    print("🚀 블록 감지 시작 (q 누르면 종료)")

    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        depth_array = np.asanyarray(depth_frame.get_data())

        # 거리 계산
        d1 = get_min_distance(depth_array, roi1)
        d2 = get_min_distance(depth_array, roi2)
        d3 = get_min_distance(depth_array, roi3)

        # -------------------------------
        # Depth 필터
        # -------------------------------

        hist1.append(d1)
        hist2.append(d2)
        hist3.append(d3)

        if len(hist1) > FILTER_SIZE:
            hist1.pop(0)
            hist2.pop(0)
            hist3.pop(0)

        d1 = np.mean(hist1)
        d2 = np.mean(hist2)
        d3 = np.mean(hist3)

        # -------------------------------
        # 블록 판단
        # -------------------------------

        s1 = 'a' if d1 > FLOOR_1_THRESH else 'A'
        s2 = 'b' if d2 > FLOOR_1_THRESH else 'B'

        # 2층은 1층 둘다 있어야 검사
        if s1 == 'A' and s2 == 'B':
            s3 = 'c' if d3 > FLOOR_2_THRESH else 'C'
        else:
            s3 = 'c'

        # -------------------------------
        # 시리얼 전송
        # -------------------------------

        if ser:

            if s1 != last_s1:
                ser.write(s1.encode())
                state = "없음(ON)" if s1 == 'a' else "있음(OFF)"
                print(f"A 좌측: {d1:.3f}m → {state}")
                last_s1 = s1

            if s2 != last_s2:
                ser.write(s2.encode())
                state = "없음(ON)" if s2 == 'b' else "있음(OFF)"
                print(f"B 우측: {d2:.3f}m → {state}")
                last_s2 = s2

            if s3 != last_s3:
                ser.write(s3.encode())
                state = "없음(ON)" if s3 == 'c' else "있음(OFF)"
                print(f"C 중앙: {d3:.3f}m → {state}")
                last_s3 = s3

        # -------------------------------
        # 화면 표시
        # -------------------------------

        depth_color = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_array, alpha=0.03),
            cv2.COLORMAP_JET
        )

        rois = [roi1, roi3, roi2]
        dists = [d1, d3, d2]
        signals = [s1, s3, s2]
        labels = ["A", "C", "B"]

        for (x, y, w, h), d, s, label in zip(rois, dists, signals, labels):

            color = (0,255,0) if s.islower() else (0,0,255)

            cv2.rectangle(depth_color, (x,y), (x+w,y+h), color, 2)

            cv2.putText(depth_color,
                        f"{label}:{d:.3f}m",
                        (x+5,y+20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2)

        cv2.imshow("RealSense Depth", depth_color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:

    pipeline.stop()
    cv2.destroyAllWindows()

    if ser:
        ser.close()
