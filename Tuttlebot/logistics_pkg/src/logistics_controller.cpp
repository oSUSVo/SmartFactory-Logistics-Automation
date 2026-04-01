#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class LogisticsController : public rclcpp::Node {
public:
    LogisticsController() : Node("logistics_controller"), current_state_(State::IDLE) {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        initWaypoints();

        // 서버 설정: IP 10.10.141.44 / PORT 9090
        tcp_thread_ = std::thread(&LogisticsController::tcpCommunicationThread, this, "10.10.141.44", 9090);
    }

    ~LogisticsController() {
        if (tcp_thread_.joinable()) tcp_thread_.join();
        if (sock_ != -1) close(sock_);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::thread tcp_thread_;
    int sock_ = -1;
    geometry_msgs::msg::PoseStamped wp_load_, wp_mid1_, wp_mid2_, wp_rail_, wp_mid3_, wp_mid4_;

    enum class State {
        IDLE,
        TO_RAIL_1, TO_RAIL_2, TO_RAIL_FINAL,
        TO_LOAD_1, TO_LOAD_2, TO_LOAD_FINAL
    };
    State current_state_;

    void initWaypoints() {
        auto set_pose = [](geometry_msgs::msg::PoseStamped& wp, double x, double y, double z, double w) {
            wp.header.frame_id = "map";
            wp.pose.position.x = x; wp.pose.position.y = y;
            wp.pose.orientation.z = z; wp.pose.orientation.w = w;
        };
        // 직접 찍은 좌표 데이터들 적용
        set_pose(wp_load_, 0.049, 0.001, -0.007, 1.000); 
        set_pose(wp_mid1_, 0.443, 0.114, 0.707, 0.707);  
        set_pose(wp_mid2_, 0.429, 0.568, 0.711, 0.703);  
        set_pose(wp_rail_, 0.490, 1.033, 0.701, 0.713);  
        set_pose(wp_mid3_, 0.289, 0.814, 1.000, -0.009); 
        set_pose(wp_mid4_, 0.226, 0.110, -0.708, 0.706); 
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& target_pose) {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 서버를 찾을 수 없습니다!");
            return;
        }
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;
        goal_msg.pose.header.stamp = this->get_clock()->now();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&LogisticsController::resultCallback, this, std::placeholders::_1);
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void resultCallback(const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            switch (current_state_) {
                case State::TO_RAIL_1: current_state_ = State::TO_RAIL_2; sendGoal(wp_mid2_); break;
                case State::TO_RAIL_2: current_state_ = State::TO_RAIL_FINAL; sendGoal(wp_rail_); break;
                case State::TO_RAIL_FINAL: 
                    current_state_ = State::IDLE; 
                    RCLCPP_INFO(this->get_logger(), "레일 도착 완료! A2 보고 중...");
                    sendTcpSignal("A2"); // 서버가 하역 작업(2초 대기)을 시작하도록 신호 전송
                    break;

                case State::TO_LOAD_1: current_state_ = State::TO_LOAD_2; sendGoal(wp_mid4_); break;
                case State::TO_LOAD_2: current_state_ = State::TO_LOAD_FINAL; sendGoal(wp_load_); break;
                case State::TO_LOAD_FINAL: 
                    current_state_ = State::IDLE; 
                    RCLCPP_INFO(this->get_logger(), "적재소 복귀 완료! A1 보고 중...");
                    sendTcpSignal("A1"); // 서버가 적재 작업(2초 대기)을 시작하도록 신호 전송
                    break;
                default: break;
            }
        }
    }

    void tcpCommunicationThread(const std::string& server_ip, int port) {
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr);

        RCLCPP_INFO(this->get_logger(), "서버(%s:%d) 연결 시도 중...", server_ip.c_str(), port);

        while (rclcpp::ok() && connect(sock_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        RCLCPP_INFO(this->get_logger(), "✅ 연결 성공! 첫 도착 신호(A1)를 보냅니다.");
        sendTcpSignal("A1"); // 연결되자마자 적재소에 있다고 알림

        char buffer[1024] = {0};
        while (rclcpp::ok()) {
            int valread = read(sock_, buffer, 1024);
            if (valread > 0) {
                std::string signal(buffer, valread);
                signal.erase(signal.find_last_not_of(" \n\r\t") + 1);
                RCLCPP_INFO(this->get_logger(), "📥 서버 명령 수신: %s", signal.c_str());

                if (signal == "L" && current_state_ == State::IDLE) {
                    RCLCPP_INFO(this->get_logger(), "적재 완료 신호 수신. 레일로 출발합니다.");
                    current_state_ = State::TO_RAIL_1;
                    sendGoal(wp_mid1_);
                } 
                else if (signal == "R" && current_state_ == State::IDLE) {
                    RCLCPP_INFO(this->get_logger(), "하역 완료 신호 수신. 적재소로 복귀합니다.");
                    current_state_ = State::TO_LOAD_1;
                    sendGoal(wp_mid3_);
                }
            }
        }
    }

    void sendTcpSignal(const std::string& msg) {
        if (sock_ != -1) {
            send(sock_, msg.c_str(), msg.length(), 0);
            RCLCPP_INFO(this->get_logger(), "📤 상태 보고 완료: %s", msg.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogisticsController>());
    rclcpp::shutdown();
    return 0;
}
