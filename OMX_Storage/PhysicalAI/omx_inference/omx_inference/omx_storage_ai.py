# omx_storage_ai.py
# 입력:  /pub (Int32)
# 출력:  /sub (Int32)
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

        self.declare_parameter(
            'model_path',
            '/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/omx_f_6_floor_032913342_S_02/checkpoints/050000/pretrained_model'
        )
        self.declare_parameter('step_hz',      30.0)
        self.declare_parameter('camera1_topic', '/storage/camera1/image_raw/compressed')
        self.declare_parameter('camera2_topic', '/storage/camera2/image_raw/compressed')
        self.declare_parameter('max_retry', 10)

        self.model_path   = self.get_parameter('model_path').value
        self.step_hz      = self.get_parameter('step_hz').value
        self.camera1_topic = self.get_parameter('camera1_topic').value
        self.camera2_topic = self.get_parameter('camera2_topic').value
        self.max_retry    = self.get_parameter('max_retry').value

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

        self.bridge        = CvBridge()
        self.lock          = threading.Lock()
        self.latest_image1 = None
        self.latest_image2 = None
        self.latest_joints = None
        self.is_running    = False

        self.target_action = None
        self.publish_active = False

        # 타이머용 변수들
        self.is_monitoring = False
        self.detection_active = False
        self.success_achieved = False
        self.detect_start_time = 0.0 # 4초 측정을 위한 시작 시간

        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'gripper_joint_1',
        ]
        self.joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']

        self.create_subscription(Int32, 'pub', self.command_callback, 10)
        self.create_subscription(CompressedImage, self.camera1_topic, self.image1_callback, 10)
        self.create_subscription(CompressedImage, self.camera2_topic, self.image2_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(Int32, '/check/result', self.check_result_callback, 10)

        self.result_pub = self.create_publisher(Int32, 'sub', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, 'gpio_command_controller/commands', 10)
        self.task_done_pub = self.create_publisher(Int32, '/check/request', 10)

        timer_period = 1.0 / self.step_hz
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)
        
        # 0.1초마다 백그라운드에서 센서만 전담해서 확인하는 타이머
        self.monitor_timer = self.create_timer(0.1, self.background_monitoring_loop)

        self.get_logger().info("OMX Storage Inference Node Ready!")

    # ════════════════════════════════════════════════════
    # 4초 정밀 측정 타이머 로직
    # ════════════════════════════════════════════════════
    def background_monitoring_loop(self):
        if not self.is_monitoring:
            self.detect_start_time = 0.0
            return

        if self.detection_active:
            # 처음 감지된 순간 시간 기록
            if self.detect_start_time == 0.0:
                self.detect_start_time = time.time()
                self.get_logger().info("👀 센서 감지 시작! 4초 카운트다운...")
            
            # 정확한 경과 시간 계산
            elapsed = time.time() - self.detect_start_time
            
            # 로그 조절 (대략 1초마다 출력)
            if int(elapsed * 10) % 10 == 0 and elapsed > 0.1:
                self.get_logger().info(f"⏳ 감지 유지 중... {elapsed:.1f}초")
                
            # 4초 도달 시 성공 판정
            if elapsed >= 5.0:
                self.success_achieved = True
                self.is_monitoring = False
                self.get_logger().info("✅ 4초 연속 감지 완료! 추론을 조기 종료합니다.")
        else:
            # 센서가 끊기면 얄짤없이 0초로 초기화
            if self.detect_start_time > 0.0:
                self.get_logger().warn("⚠️ 감지 끊김. 타이머를 리셋합니다.")
            self.detect_start_time = 0.0

    def control_timer_callback(self):
        with self.lock:
            if not self.publish_active or self.target_action is None:
                return
            action_to_publish = self.target_action.copy()

        self.publish_arm_trajectory(action_to_publish)
        self.publish_gripper_gpio(float(action_to_publish[5]))

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

    def check_result_callback(self, msg: Int32):
        # 1이 들어오면 True, 0이면 False
        self.detection_active = (msg.data == 1)

    def execute_with_retry(self, goal_num: int):
        self.is_running = True
        self.move_home(keep_timer=True)

        for attempt in range(self.max_retry):
            self.get_logger().info(f"🔄 [시도 {attempt+1}/{self.max_retry}] 작업 시작")
            self.publish_active = True
            
            self.success_achieved = False
            self.detection_active = False
            self.detect_start_time = 0.0
            self.is_monitoring = True
            
            self.task_done_pub.publish(Int32(data=1))

            result = self.run_single_inference()
            
            self.is_monitoring = False

            if result == 'success':
                self.get_logger().info("🎉 짐 옮기기 및 4초 안착 확인 최종 성공!")
                self.success_achieved = False  # 복귀 추론이 바로 종료되지 않도록
                self.is_monitoring = False
                self.run_single_inference(duration=10.0)  # 10초 복귀 추론
                self.publish_active = False
                self.send_result(1)
                self.move_home()
                self.is_running = False  # ⭐️ 복귀 추론 끝난 후에 False
                return

            if result == 'error':
                self.get_logger().error("추론 중 오류 발생, 재시도")
                continue

            self.get_logger().warn(f"❌ {attempt+1}차 시도 실패 (제한 시간 초과)")

        self.publish_active = False
        self.send_result(0)
        self.move_home()
        self.is_running = False

    def run_single_inference(self, duration=10.0):
        rate_sec = 1.0 / self.step_hz
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < duration:  # ⭐️ duration 파라미터
                if not self.is_running or self.success_achieved:
                    return 'success' if self.success_achieved else 'failed'

                t_step_start = time.time() 

                observation = self.get_observation()
                if observation is None:
                    time.sleep(0.01)
                    continue

                with torch.no_grad():
                    action = self.policy.select_action(observation).cpu().numpy().flatten()

                with self.lock:
                    self.target_action = action

                elapsed = time.time() - t_step_start
                sleep_time = rate_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            return 'timeout' 

        except Exception as e:
            self.get_logger().error(f"추론 오류: {e}")
            return 'error'

    def move_home(self, keep_timer=False):
        if not keep_timer:
            self.control_timer.cancel()
        with self.lock:
            self.publish_active = False 
            self.get_logger().info("Move to HOME")
        
        # 2. 그리퍼 먼저 열기 (GPIO 명령 전송)
        self.publish_gripper_gpio(0.015) 
        time.sleep(1.0) 
        
        # 4. 홈 포즈 설정
        # [joint1, joint2, joint3, joint4, joint5, gripper]
        target_pose = [0.0, -1.57, 1.57, 1.57, 0.0, 0.015] 

        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = target_pose
        point.time_from_start = Duration(sec=4, nanosec=0)
        msg.points = [point]
        
        self.arm_pub.publish(msg)
        time.sleep(4.5)

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
        self.get_logger().info(f"최종 결과 송신: {result_data} (1=성공, 0=실패)")


def main():
    rclpy.init()
    node = OMXStorageInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()