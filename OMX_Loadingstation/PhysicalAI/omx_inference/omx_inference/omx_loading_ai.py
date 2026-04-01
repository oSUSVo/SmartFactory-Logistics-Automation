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
            '/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/merge_6_floor_03292012_02/checkpoints/043000/pretrained_model'
        )
        self.declare_parameter('step_hz',      30.0)
        # ⭐️ max_steps 대신 직관적인 시간 제한 파라미터 도입
        self.declare_parameter('timeout_sec',  3.5) 
        self.declare_parameter('camera1_topic', '/loading/camera1/image_raw/compressed')
        self.declare_parameter('camera2_topic', '/loading/camera2/image_raw/compressed')
        self.declare_parameter('max_retry', 20)

        self.model_path    = self.get_parameter('model_path').value
        self.step_hz       = self.get_parameter('step_hz').value
        self.timeout_sec   = self.get_parameter('timeout_sec').value
        self.camera1_topic = self.get_parameter('camera1_topic').value
        self.camera2_topic = self.get_parameter('camera2_topic').value
        self.max_retry     = self.get_parameter('max_retry').value

        self.home_joints = np.array([0.0, -1.57, 1.57, 1.57, 0.0, 0.02])

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

        # 제어용 변수 추가
        self.target_action = None
        self.publish_active = False

        # ── 조인트 이름 ───────────────────────────────────────────
        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'gripper_joint_1',
        ]
        self.joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']

        # ── Subscriber ────────────────────────────────────────────
        self.create_subscription(Int32, 'pub', self.command_callback, 10)
        self.create_subscription(CompressedImage, self.camera1_topic, self.image1_callback, 10)
        self.create_subscription(CompressedImage, self.camera2_topic, self.image2_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # ── Publisher ─────────────────────────────────────────────
        self.result_pub = self.create_publisher(Int32, 'sub', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, 'gpio_command_controller/commands', 10)

        # 제어 타이머 생성 (추론과 무관하게 30Hz로 퍼블리시 전담)
        timer_period = 1.0 / self.step_hz
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)

        self.get_logger().info("OMX Loading Inference Node Ready!")
        self.get_logger().info(f"추론 주기    : {self.step_hz} Hz")
        self.get_logger().info(f"동작 제한시간 : {self.timeout_sec} 초")

    # ════════════════════════════════════════════════════
    # 비동기 제어 타이머 루프
    # ════════════════════════════════════════════════════
    def control_timer_callback(self):
        with self.lock:
            if not self.publish_active or self.target_action is None:
                return
            action_to_publish = self.target_action.copy()

        self.publish_arm_trajectory(action_to_publish)
        self.publish_gripper_gpio(float(action_to_publish[5]))

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
            self.control_timer.reset()  # ⭐️ reset을 여기서 — 다음 명령 수신 시점에
            t = threading.Thread(
                target=self.execute_with_retry,
                args=(msg.data,),
                daemon=True
            )
            t.start()

    # ----------------------------------------------------------------------------------------
    # 추론 시작
    def execute_with_retry(self, goal_num: int):
        self.is_running = True
        self.move_home(keep_timer=True)

        for attempt in range(self.max_retry):
            # 먼저 RGB 확인
            if self.check_cargo():
                self.get_logger().info("✅ 성공 (짐 감지 완료)")
                self.publish_active = True
                self.execute_inference(goal_num, duration=10.0)  # ⭐️ 10초 복귀 추론
                self.publish_active = False
                self.is_running = False
                self.send_result(1)
                self.move_home()
                return

            # 짐 없으면 AI 추론 실행
            self.get_logger().info(f"실행 ({attempt+1}/{self.max_retry})")
            
            self.publish_active = True 
            self.execute_inference(goal_num)
            self.publish_active = False 

        # 모든 시도 종료 후 최종 확인
        if self.check_cargo():
            self.get_logger().info("✅ 성공 (마지막 확인)")
            self.publish_active = True
            self.execute_inference(goal_num, duration=10.0)  # ⭐️ 10초 복귀 추론
            self.publish_active = False
            self.is_running = False
            self.send_result(1)
            self.move_home()
            return 

        # 실패 시
        self.get_logger().error("❌ 실패")
        self.is_running = False
        self.send_result(0)
        self.move_home()

    # 색 확인
    def check_cargo(self):
        with self.lock:
            if self.latest_image1 is None:
                self.get_logger().warn("Cannot Find Camera")
                return False
            img = self.latest_image1.copy()

        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        y_start, y_end = 100, 100 + 180 
        x_start, x_end = 450, 450 + 180
        roi = hsv_img[y_start:y_end, x_start:x_end]

        lower_wood = np.array([160, 100, 100])
        upper_wood = np.array([179, 255, 255])

        mask = cv2.inRange(roi, lower_wood, upper_wood)
        wood_pixels = cv2.countNonZero(mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        wood_ratio = wood_pixels / total_pixels

        self.get_logger().info(f"짐 확인 중... (픽셀 비율: {wood_ratio*100:.1f}%)")

        if wood_ratio > 0.03:
            return True
        else:
            return False

    # 추론
    def execute_inference(self, goal_num: int, duration: float = None):
        rate_sec = 1.0 / self.step_hz
        start_time = time.time()
        step = 0
        timeout = duration if duration is not None else self.timeout_sec

        try:
            while (time.time() - start_time) < timeout:
                if not self.is_running:
                    return

                t_step_start = time.time()
                observation = self.get_observation()
                if observation is None:
                    self.get_logger().warn("Observation 미수신 — 대기")
                    time.sleep(0.01)
                    continue

                with torch.no_grad():
                    action = self.policy.select_action(observation).cpu().numpy().flatten()

                with self.lock:
                    self.target_action = action

                if step % 15 == 0:
                    self.get_logger().info(
                        f"[동작 중... {time.time()-start_time:.1f}초 경과] "
                        f"arm={np.round(action[:5], 3).tolist()}"
                    )

                elapsed = time.time() - t_step_start
                sleep_time = rate_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                step += 1

        except Exception as e:
            self.get_logger().error(f"추론 오류: {e}")
            

    # ----------------------------------------------------------------------------------------
    # Observation / Publish 헬퍼
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
        
    # ----------------------------------------------------------------------------------------
    # 끝났을 경우 강제 홈으로
    def move_home(self, keep_timer=False):
        if not keep_timer:
            self.control_timer.cancel()
        with self.lock:
            self.publish_active = keep_timer  # ⭐️ keep_timer 반영
            self.target_action = self.home_joints.copy()
            self.get_logger().info("Move to HOME")

        self.publish_gripper_gpio(0.15)
        time.sleep(1.0)

        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_joints.tolist()
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points = [point]
        self.arm_pub.publish(msg)
        time.sleep(3.5)

def main():
    rclpy.init()
    node = OMXLoadingInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()