# omx_storage_ai.py
# 입력:  /omx_storage_pub (Int32)
# 출력:  /omx_storage_sub (Int32)
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

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig
from huggingface_hub.constants import CONFIG_NAME


class OMXStorageInferenceNode(Node):
    def __init__(self):
        super().__init__('omx_storage_inference_node')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            '/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/test/checkpoints/010000/pretrained_model'
        )
        self.declare_parameter('step_hz',      30.0)
        self.declare_parameter('max_steps',    10000)
        self.declare_parameter('camera_topic', 'camera1/image_raw/compressed')

        self.model_path   = self.get_parameter('model_path').value
        self.step_hz      = self.get_parameter('step_hz').value
        self.max_steps    = self.get_parameter('max_steps').value
        self.camera_topic = self.get_parameter('camera_topic').value

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
        self.latest_image  = None
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
            CompressedImage, self.camera_topic,
            self.image_callback, 10
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

        self.get_logger().info("OMX Storage Inference Node Ready!")
        self.get_logger().info(f"카메라 토픽  : {self.camera_topic}")
        self.get_logger().info(f"추론 주기    : {self.step_hz} Hz")
        self.get_logger().info(f"최대 스텝    : {self.max_steps}")

    # ════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════

    def image_callback(self, msg: CompressedImage):
        with self.lock:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'rgb8')

    def joint_callback(self, msg: JointState):
        with self.lock:
            joint_map = dict(zip(msg.name, msg.position))
            joints = [joint_map[n] for n in self.joint_order if n in joint_map]
            self.latest_joints = joints if len(joints) == 6 else None

    def command_callback(self, msg: Int32):
        if msg.data >= 1 and not self.is_running:
            self.get_logger().info(f"▶ 명령 수신 goal_num: {msg.data}")
            t = threading.Thread(
                target=self.execute_inference,
                args=(msg.data,),
                daemon=True
            )
            t.start()
        elif self.is_running:
            self.get_logger().warn("이미 실행 중 — 신호 무시")

    # ════════════════════════════════════════════════════
    # 추론 루프
    # ════════════════════════════════════════════════════

    def execute_inference(self, goal_num: int):
        self.is_running = True
        result_data = 0
        rate_sec = 1.0 / self.step_hz

        try:
            for step in range(self.max_steps):
                if not self.is_running:
                    break

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

            result_data = 1

        except Exception as e:
            self.get_logger().error(f"추론 오류: {e}")
            result_data = 0

        finally:
            self.is_running = False
            self.send_result(result_data)

    # ════════════════════════════════════════════════════
    # Observation / Publish 헬퍼
    # ════════════════════════════════════════════════════

    def get_observation(self):
        with self.lock:
            if self.latest_image is None or self.latest_joints is None:
                return None
            device = next(self.policy.parameters()).device
            return {
                'observation.images.camera1': torch.from_numpy(
                    self.latest_image.copy()
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
        self.get_logger().info(f"■ 결과 송신: {result_data} (1=성공, 0=실패)")


def main():
    rclpy.init()
    node = OMXStorageInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()