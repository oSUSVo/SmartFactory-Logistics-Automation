#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class WaypointTester : public rclcpp::Node {
public:
    WaypointTester() : Node("waypoint_tester_node"), current_wp_index_(0) {
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        setup_waypoints();

        RCLCPP_INFO(this->get_logger(), "Nav2 서버를 기다리는 중...");
        nav_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Nav2 서버 연결 완료! 4개의 웨이포인트 주행 테스트를 시작합니다.");

        send_next_goal();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_wp_index_;

    void setup_waypoints() {
        geometry_msgs::msg::PoseStamped wp;
        wp.header.frame_id = "map";

        // [WP 1] 적재소 (Loading)
        wp.pose.position.x = 0.009768; wp.pose.position.y = 0.064961;
        wp.pose.orientation.z = -0.005912; wp.pose.orientation.w = 0.999982;
        waypoints_.push_back(wp);

        // [WP 2] way1 (갈 때 경유지)
        wp.pose.position.x = 0.615384; wp.pose.position.y = 0.502122;
        wp.pose.orientation.z = 0.695853; wp.pose.orientation.w = 0.718183;
        waypoints_.push_back(wp);

        // [WP 3] 레일 (Rail / 수확지점)
        wp.pose.position.x = 0.736064; wp.pose.position.y = 1.080588;
        wp.pose.orientation.z = -0.711439; wp.pose.orientation.w = 0.702747;
        waypoints_.push_back(wp);

        // [WP 4] way2 (올 때 경유지)
        wp.pose.position.x = 0.195380; wp.pose.position.y = 0.846903;
        wp.pose.orientation.z = -0.720624; wp.pose.orientation.w = 0.693325;
        waypoints_.push_back(wp);
    }

    void send_next_goal() {
        if (current_wp_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "🎉 4개의 웨이포인트 주행 테스트를 모두 성공적으로 마쳤습니다!");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_wp_index_];
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "[WP %zu]를 향해 출발합니다!", current_wp_index_ + 1);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&WaypointTester::nav_result_callback, this, std::placeholders::_1);
        
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void nav_result_callback(const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "✅ [WP %zu] 도착 완료!", current_wp_index_ + 1);
            current_wp_index_++;
            
            // 도착 후 2초 대기했다가 다음 포인트로 출발
            rclcpp::sleep_for(std::chrono::seconds(2));
            send_next_goal();
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 주행 실패. 로봇이 경로를 찾지 못했거나 장애물에 막혔습니다.");
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
