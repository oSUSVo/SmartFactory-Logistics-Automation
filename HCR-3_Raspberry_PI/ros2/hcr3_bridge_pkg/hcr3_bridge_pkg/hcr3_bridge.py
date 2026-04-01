# hcr3_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import threading
import os

class Hcr3Bridge(Node):
    def __init__(self):
        super().__init__('hcr3_bridge')
        
        # ── 토픽 설정 ──────────────────────────────────────────
        # 'pub' 토픽 구독 (서버 시작 신호)
        self.subscription = self.create_subscription(Int32, 'pub', self.start_callback, 10)
        self.publisher = self.create_publisher(Int32, 'sub', 10)
        self.request_publisher = self.create_publisher(Int32, '/request_sub', 10)
        
        # omx_storage(AI 노드)의 센서 확인 요청 수신
        self.check_request_sub = self.create_subscription(Int32, '/check/request', self.robot_placed_callback, 10)
        
        # 적외선 센서 결과 송신 (AI 노드가 이걸 보고 4초를 계산함)
        self.check_result_pub = self.create_publisher(Int32, '/check/result', 10)

        # ── 프로세스 실행 ──────────────────────────────────────
        # -u 옵션: 실시간 출력을 위해 버퍼링 방지
        self.process = subprocess.Popen(
            ['python3', '-u', '/home/ubuntu/relay/depth_control2.py'], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        # 비전 출력을 실시간 감시할 스레드
        self.thread = threading.Thread(target=self.monitor_output, daemon=True)
        self.thread.start()
        
        self.get_logger().info("HCR3 Bridge Node Started. Python -u enabled.")

    # ── 추가된/복구된 함수들 ───────────────────────────────────

    def start_callback(self, msg):
        """서버 시작 신호를 비전 프로세스에 전달"""
        self.get_logger().info(f"🚀 서버 시작 신호 수신: {msg.data}")
        self.send_to_vision("START")

    def robot_placed_callback(self, msg):
        """AI 노드의 안착 신호를 비전에 전달"""
        if msg.data == 1:
            self.get_logger().info("🤖 로봇 팔 안착 완료 -> 비전에 판정 요청")
            self.send_to_vision("CHECK_IR")

    def send_to_vision(self, command):
        """subprocess의 stdin을 통해 비전 파이썬 파일로 명령 전송"""
        try:
            self.process.stdin.write(f"{command}\n")
            self.process.stdin.flush()
            self.get_logger().info(f"➡️ depth_control 명령 전송: {command}")
        except Exception as e:
            self.get_logger().error(f"❌ 비전 프로세스 통신 실패: {e}")

    def monitor_output(self):
        """비전 노드의 출력을 실시간 감시하며 ROS 토픽 발행"""
        for line in iter(self.process.stdout.readline, ''):
            clean_line = line.strip()
            if not clean_line: continue
            
            # 1. 실시간 감지 상태 중계 (AI 노드 전달용)
            # 이 부분은 로그 없이 조용히 발행만 하는 것이 좋습니다.
            if "[IR_STATE] 1" in clean_line:
                self.check_result_pub.publish(Int32(data=1))
            elif "[IR_STATE] 0" in clean_line:
                self.check_result_pub.publish(Int32(data=0))

            # 2. 물건 요청 처리 (중요하니까 로그 유지)
            elif "[REQUEST_BOX] 1" in clean_line:
                self.get_logger().info("📦 물건 요청 신호 감지!")
                self.request_publisher.publish(Int32(data=1))

            # 3. 비전 쪽 자체 판정 성공 시 (일종의 '프리패스' 신호)
            elif "[CHECK_SUCCESS]" in clean_line:
                self.get_logger().info("✅ 비전 노드로부터 최종 성공 확인 수신")
                self.check_result_pub.publish(Int32(data=1))

            # 4. 전체 공정 완료 보고
            elif "🎉 모든 칸 완료!" in clean_line:
                self.get_logger().info("🎯 모든 공정 완료!")
                self.publisher.publish(Int32(data=1))
                self.send_to_vision("STOP_CHECK")

    def destroy_node(self):
        self.process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Hcr3Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()