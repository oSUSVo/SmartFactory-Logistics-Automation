#include <functional>
#include <memory>
#include <thread>
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
 
#include "custom_interfaces/action/move.hpp"
#include "geometry_msgs/msg/twist.hpp"
 
 
class MyActionServer : public rclcpp::Node {
public:
  using Move = custom_interfaces::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
 
  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("smart_factory_server", options)
  {
    using namespace std::placeholders;
 
    this->action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move_robot_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1)
    );
 
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
 
  }
 
private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
 
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->secs);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
 
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // executor를 막지 않고 빠르게 리턴 해야합니다.
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }
 
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto & message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Move::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
 
    for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
      // 취소 요청이 있는지 확인
      if (goal_handle->is_canceling()) {
        result->status = message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // 로봇 움직이면서 feedback 보내기
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
 
      loop_rate.sleep();
    }
 
    // goal이 끝났으면
    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class MyActionServer
 
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
 
  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
 
  rclcpp::shutdown();
  return 0;
}