# omx_loading_ai.py
# 입력:  /omx_loading_pub (Int32)
# 출력:  /omx_loading_sub (Int32)
# 실행환경: physical_ai_tools 도커 (Ubuntu 24.04 + ROS2 Jazzy)

import os
import json
import tempfile
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import threading
import numpy as np
import torch
import draccus
import time
import cv2

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig
from huggingface_hub.constants import CONFIG_NAME


class OMXLoadingInferenceNode(Node):
    def __init__(self):
        super().__init__('omx_loading_inference_node')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            '/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/omx_f_4f_cam2_3box_0318_1830/checkpoints/040000/pretrained_model'
        )
        self.declare_parameter('step_hz',      30.0)
        self.declare_parameter('max_steps',    750)
        self.declare_parameter('camera1_topic', 'camera1/image_raw/compressed')
        self.declare_parameter('camera2_topic', 'camera2/image_raw/compressed')
        self.declare_parameter('max_retry', 3)

        self.model_path   = self.get_parameter('model_path').value
        self.step_hz      = self.get_parameter('step_hz').value
        self.max_steps    = self.get_parameter('max_steps').value
        self.camera1_topic = self.get_parameter('camera1_topic').value
        self.camera2_topic = self.get_parameter('camera2_topic').value
        self.max_retry = self.get_parameter('max_retry').value

        # ── 모델 로드 ─────────────────────────────────────────────
        config_file = os.path.join(self.model_path, CONFIG_NAME)

        with open(config_file) as f:
            config_dict = json.load(f)
        config_dict.pop("type", None)

        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as f:
            json.dump(config_dict, f)
            tmp_path = f.name

        with draccus.config_type("json"):
            cfg = draccus.parse(ACTConfig, tmp_path, args=[])
        cfg.pretrained_path = self.model_path
        cfg.device = 'cuda'

        self.policy = ACTPolicy.from_pretrained(self.model_path, config=cfg)
        self.policy.eval()
        self.get_logger().info(f"모델 로드 완료: {self.model_path}")

        # ── 유틸 ──────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.lock          = threading.Lock()
        self.latest_image1 = None
        self.latest_image2 = None
        self.latest_joints = None
        self.is_running    = False

        # ── 조인트 이름 ───────────────────────────────────────────
        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'gripper_joint_1',
        ]
        self.joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']

        # ── Subscriber ────────────────────────────────────────────
        self.create_subscription(
            Int32, 'pub',
            self.command_callback, 10
        )
        self.create_subscription(
            CompressedImage, self.camera1_topic,
            self.image1_callback, 10
        )
        self.create_subscription(
            CompressedImage, self.camera2_topic,
            self.image2_callback, 10
        )
        self.create_subscription(
            JointState, 'joint_states',
            self.joint_callback, 10
        )

        # ── Publisher ─────────────────────────────────────────────
        self.result_pub = self.create_publisher(
            Int32, 'sub', 10
        )
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            'arm_controller/joint_trajectory', 10
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            'gpio_command_controller/commands', 10
        )

        self.get_logger().info("OMX Loading Inference Node Ready!")
        self.get_logger().info(f"카메라1 토픽 : {self.camera1_topic}")
        self.get_logger().info(f"카메라2 토픽 : {self.camera2_topic}")
        self.get_logger().info(f"추론 주기    : {self.step_hz} Hz")
        self.get_logger().info(f"최대 스텝    : {self.max_steps}")

    # ════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════

    def image1_callback(self, msg: CompressedImage):
        with self.lock:
            self.latest_image1 = self.bridge.compressed_imgmsg_to_cv2(msg, 'rgb8')

    def image2_callback(self, msg: CompressedImage):
        with self.lock:
            self.latest_image2 = self.bridge.compressed_imgmsg_to_cv2(msg, 'rgb8')

    def joint_callback(self, msg: JointState):
        with self.lock:
            joint_map = dict(zip(msg.name, msg.position))
            joints = [joint_map[n] for n in self.joint_order if n in joint_map]
            self.latest_joints = joints if len(joints) == 6 else None

    def command_callback(self, msg: Int32):
        if msg.data >= 1 and not self.is_running:
            self.get_logger().info(f"▶ 명령 수신 goal_num: {msg.data}")
            t = threading.Thread(
                target=self.execute_with_retry,
                args=(msg.data,),
                daemon=True
            )
            t.start()
        elif self.is_running:
            self.get_logger().warn("이미 실행 중 — 신호 무시")

    # ════════════════════════════════════════════════════
    # 추론 루프
    # ════════════════════════════════════════════════════

    def execute_with_retry(self, goal_num: int):
        self.is_running = True

        for attempt in range(self.max_retry):
            # 먼저 RGB 확인
            if self.check_cargo():
                self.get_logger().info("✅ 짐 확인 → 성공")
                self.is_running = False
                self.send_result(1)
                return

            # 짐 없으면 AI 추론 실행
            self.get_logger().info(f"🔄 AI 실행 ({attempt+1}/{self.max_retry})")
            self.execute_inference(goal_num)

        # 마지막 한 번 더 RGB 확인
        if self.check_cargo():
            self.get_logger().info("✅ 짐 확인 → 성공")
            self.is_running = False
            self.send_result(1)
            return 

        # 3회 모두 실패
        self.get_logger().error("❌ 3회 실패 → 실패 송신")
        self.is_running = False
        self.send_result(0)

    def check_cargo(self):
        with self.lock:
            if self.latest_image1 is None:
                self.get_logger().warn("Cannot Find Camera")
                return False
            # cv_bridge로 변환한 이미지는 RGB이므로 OpenCV 처리를 위해 복사
            img = self.latest_image1.copy()

        # RGB -> HSV 색공간 변환
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        # 관심영역 설정
        # 시작점: x=510, y=140
        # 넓이: w=110, h=120
        # Numpy 슬라이싱 순서 [y_start : y_end, x_start : x_end]
        y_start, y_end = 140, 140 + 120 
        x_start, x_end = 510, 510 + 110  # 510 ~ 620
        roi = hsv_img[y_start:y_end, x_start:x_end]

        # 실측값 [14, 100, 153]
        # Hue: 14 -> 범위를 7 ~ 22 (짐 색깔)
        # Saturation: 100 -> 범위를 70 ~ 150 (색의 진하기)
        # Value: 153 -> 범위를 80 ~ 220 (밝기 변화 대응)
        lower_wood = np.array([7, 70, 80])
        upper_wood = np.array([22, 150, 220])

        # 해당 색상 영역 마스크 생성 및 픽셀 수 계산
        mask = cv2.inRange(roi, lower_wood, upper_wood)
        wood_pixels = cv2.countNonZero(mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        wood_ratio = wood_pixels / total_pixels

        self.get_logger().info(f"짐 확인 중... (픽셀 비율: {wood_ratio*100:.1f}%)")

        # 짐의 색 픽셀이 ROI 면적의 5% 이상이면 짐이 있는 것으로 간주
        if wood_ratio > 0.05:
            return True
        else:
            return False


    def execute_inference(self, goal_num: int):
        # is_running은 execute_with_retry에서 관리하므로 여기선 제거
        result_data = 0
        rate_sec = 1.0 / self.step_hz

        try:
            for step in range(self.max_steps):
                t_start = time.time()
                observation = self.get_observation()
                if observation is None:
                    self.get_logger().warn(f"[Step {step+1}] Observation 미수신 — 대기")
                    time.sleep(rate_sec)
                    continue

                with torch.no_grad():
                    action = self.policy.select_action(observation).cpu().numpy().flatten()

                self.publish_arm_trajectory(action)
                self.publish_gripper_gpio(float(action[5]))

                self.get_logger().info(
                    f"[Step {step+1:>5}/{self.max_steps}] "
                    f"arm={np.round(action[:5], 3).tolist()} "
                    f"gripper={action[5]:.3f}"
                )

                elapsed = time.time() - t_start
                sleep_time = rate_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            self.get_logger().error(f"추론 오류: {e}")

    # ════════════════════════════════════════════════════
    # Observation / Publish 헬퍼
    # ════════════════════════════════════════════════════

    def get_observation(self):
        with self.lock:
            if self.latest_image1 is None or self.latest_image2 is None or self.latest_joints is None:
                return None
            device = next(self.policy.parameters()).device
            return {
                'observation.images.camera1': torch.from_numpy(
                    self.latest_image1.copy()
                ).permute(2, 0, 1).float().unsqueeze(0).to(device) / 255.0,

                'observation.images.camera2': torch.from_numpy(
                    self.latest_image2.copy()
                ).permute(2, 0, 1).float().unsqueeze(0).to(device) / 255.0,

                'observation.state': torch.tensor(
                    self.latest_joints[:6], dtype=torch.float32
                ).unsqueeze(0).to(device),
            }

    def publish_arm_trajectory(self, action: np.ndarray):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = action[:6].tolist()
        point.time_from_start = Duration(sec=0, nanosec=int(1e9 / self.step_hz))
        msg.points = [point]
        self.arm_pub.publish(msg)

    def publish_gripper_gpio(self, gripper_val: float):
        msg = Float64MultiArray()
        msg.data = [gripper_val]
        self.gripper_pub.publish(msg)

    def send_result(self, result_data: int):
        msg = Int32()
        msg.data = result_data
        self.result_pub.publish(msg)
        self.get_logger().info(f"결과 송신: {result_data}")


def main():
    rclpy.init()
    node = OMXLoadingInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()