import pyrealsense2 as rs
import numpy as np
import serial
import cv2
import time

# -----------------------------
# SERIAL
# -----------------------------

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("STM32 연결 성공")
except:
    ser = None
    print("STM32 연결 실패")

# -----------------------------
# 블록 정보
# -----------------------------

BLOCK_HEIGHT = 0.045

# -----------------------------
# RealSense 설정 (경량)
# -----------------------------

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth,424,240,rs.format.z16,15)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# -----------------------------
# ROI (424x240 기준)
# -----------------------------

roi1 = (110,110,40,40)   # A
roi3 = (200,110,40,40)   # C
roi2 = (290,110,40,40)   # B

# -----------------------------
# Depth 계산
# -----------------------------

def get_min_distance(depth_array,roi):

    x,y,w,h = roi

    roi_depth = depth_array[y:y+h,x:x+w]*depth_scale

    valid = roi_depth[(roi_depth>0.35) & (roi_depth<1.0)]

    if valid.size < 10:
        return 9.99

    sorted_depth = np.sort(valid)

    sample = sorted_depth[:max(5,int(len(sorted_depth)*0.05))]

    return np.mean(sample)

# -----------------------------
# 바닥 캘리브레이션
# -----------------------------

print("바닥 캘리브레이션...")

samples=[]

for i in range(40):

    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()

    depth_array = np.asanyarray(depth.get_data())

    d = get_min_distance(depth_array,roi1)

    samples.append(d)

FLOOR_DISTANCE = np.mean(samples)

print("바닥:",FLOOR_DISTANCE)

FLOOR_1_THRESH = FLOOR_DISTANCE - BLOCK_HEIGHT/2
FLOOR_2_THRESH = FLOOR_DISTANCE - BLOCK_HEIGHT*1.5

# -----------------------------
# 상태
# -----------------------------

state="CHECK_A"

detect_enable=True
robot_start_time=0

robot_work_time=4
stabilize_time=2

FILTER_SIZE=5

hist1=[]
hist2=[]
hist3=[]

# -----------------------------
# 실행
# -----------------------------

try:

    print("시스템 시작")

    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        depth_array = np.asanyarray(depth_frame.get_data())

        d1 = get_min_distance(depth_array,roi1)
        d2 = get_min_distance(depth_array,roi2)
        d3 = get_min_distance(depth_array,roi3)

        hist1.append(d1)
        hist2.append(d2)
        hist3.append(d3)

        if len(hist1)>FILTER_SIZE:
            hist1.pop(0)
            hist2.pop(0)
            hist3.pop(0)

        d1=np.mean(hist1)
        d2=np.mean(hist2)
        d3=np.mean(hist3)

        # -----------------------
        # 상태 머신
        # -----------------------

        if detect_enable:

            if state=="CHECK_A":

                if d1 > FLOOR_1_THRESH:

                    if ser: ser.write(b'a')

                    state="PLACE_A"
                    detect_enable=False
                    robot_start_time=time.time()

                    print("A 쌓기")

            elif state=="CHECK_B":

                if d2 > FLOOR_1_THRESH:

                    if ser: ser.write(b'b')

                    state="PLACE_B"
                    detect_enable=False
                    robot_start_time=time.time()

                    print("B 쌓기")

            elif state=="CHECK_C":

                if d1 < FLOOR_1_THRESH and d2 < FLOOR_1_THRESH:

                    if d3 > FLOOR_2_THRESH:

                        if ser: ser.write(b'c')

                        state="PLACE_C"
                        detect_enable=False
                        robot_start_time=time.time()

                        print("C 쌓기")

        # -----------------------
        # 로봇 완료
        # -----------------------

        if not detect_enable:

            if time.time()-robot_start_time > (robot_work_time+stabilize_time):

                detect_enable=True

                if state=="PLACE_A":
                    state="CHECK_B"

                elif state=="PLACE_B":
                    state="CHECK_C"

                elif state=="PLACE_C":
                    state="DONE"

                print("다음 단계:",state)

        # -----------------------
        # 콘솔 출력
        # -----------------------

        print(f"A:{d1:.3f} B:{d2:.3f} C:{d3:.3f} | {state}")

        time.sleep(0.03)

except KeyboardInterrupt:

    pass

finally:

    pipeline.stop()

    if ser:
        ser.close()

    print("종료")