#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class LogisticsController : public rclcpp::Node {
public:
		LogisticsController() : Node("turtlebot3_control"), current_wp_index_(0), current_state_(State::IDLE) {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        initWaypoints();

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/turtle_pub", 10, std::bind(&LogisticsController::commandCallback, this, std::placeholders::_1)
        );

        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/turtle_sub", 10);

        RCLCPP_INFO(this->get_logger(), "Logistics Controller (적재소 경유, 레일 직행 버전) 시작됨. 대기 중...");

        // 초기 상태 보고 (로봇이 처음에 적재소에 있다고 가정)
        init_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
            RCLCPP_INFO(this->get_logger(), "✅ 초기 상태(1: 적재소 대기)를 서버에 보고합니다.");
            sendStatus(1);
            init_timer_->cancel();
        });
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    // 모든 웨이포인트 변수 선언
    geometry_msgs::msg::PoseStamped wp_load_, wp_rail_, wp1_, wp2_, wp3_;
    
    // 현재 이동해야 할 경로 목록을 담는 벡터
    std::vector<geometry_msgs::msg::PoseStamped> active_path_;
    size_t current_wp_index_;

    enum class State {
        IDLE,
        TO_RAIL,
        TO_LOAD
    };
    State current_state_;

    void initWaypoints() {
        auto set_pose = [](geometry_msgs::msg::PoseStamped& wp, double x, double y, double z, double w) {
            wp.header.frame_id = "map";
            wp.pose.position.x = x; wp.pose.position.y = y; wp.pose.position.z = 0.0;
            wp.pose.orientation.x = 0.0; wp.pose.orientation.y = 0.0;
            wp.pose.orientation.z = z; wp.pose.orientation.w = w;
        };

        // 사진에서 추출한 좌표 입력
        set_pose(wp_load_, 0.181, 0.097,  0.702,  0.712); // 적재소
        set_pose(wp_rail_, 0.562, 1.072,  0.664,  0.747); // 레일
        set_pose(wp1_,     0.436, 0.713, -0.699,  0.715); // 웨이포인트 1
        set_pose(wp2_,     0.464, 0.154, -0.693,  0.721); // 웨이포인트 2
        set_pose(wp3_,     0.398, -0.008, 0.998, -0.060); // 웨이포인트 3
    }

    void commandCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "서버 명령 수신: %d", msg->data);

        // 명령 1 수신 시 -> 적재소로 이동 (경유지: wp1 -> wp2 -> wp3 -> 적재소)
        if (msg->data == 1 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 1 수신: 적재소로 순차 이동을 시작합니다.");
            current_state_ = State::TO_LOAD;
            active_path_ = {wp2_, wp3_, wp_load_};
            current_wp_index_ = 0;
            sendGoal(active_path_[current_wp_index_]);
        } 
        // 명령 2 수신 시 -> 레일로 이동 (경유지 없이 직행)
        else if (msg->data == 2 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 2 수신: 레일로 직행합니다.");
            current_state_ = State::TO_RAIL;
            active_path_ = {wp_rail_}; // 🔄 여기를 직행으로 수정했습니다!
            current_wp_index_ = 0;
            sendGoal(active_path_[current_wp_index_]);
        }
    }

    void sendStatus(int status_code) {
        auto msg = std_msgs::msg::Int32();
        msg.data = status_code;
        status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "상태 보고 완료: %d", status_code);
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
            current_wp_index_++; // 다음 인덱스로 증가

            // 아직 가야 할 목적지가 남은 경우 (적재소로 갈 때만 해당됨)
            if (current_wp_index_ < active_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "경유지 도착 완료. 다음 경유지(%zu/%zu)로 이동합니다.", 
                            current_wp_index_ + 1, active_path_.size());
                sendGoal(active_path_[current_wp_index_]);
            } 
            // 최종 목적지에 도착한 경우 (레일은 한 번에 여기로 옴)
            else {
                if (current_state_ == State::TO_LOAD) {
                    current_state_ = State::IDLE; 
                    RCLCPP_INFO(this->get_logger(), "✅ 적재소(최종) 도착 완료 (서버에 1 전송)");
                    sendStatus(1); 
                }
                else if (current_state_ == State::TO_RAIL) {
                    current_state_ = State::IDLE; 
                    RCLCPP_INFO(this->get_logger(), "✅ 레일 도착 완료 (서버에 2 전송)");
                    sendStatus(2); 
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "이동 실패! 로봇이 정지되었거나 경로를 찾을 수 없습니다.");
            current_state_ = State::IDLE; // 실패 시 다시 명령을 받을 수 있도록 초기화
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogisticsController>());
    rclcpp::shutdown();
    return 0;
}