#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/int_act.hpp"

using namespace std::placeholders;

class OmxLoadingAction : public rclcpp::Node {
public:
    using IntAct = custom_interfaces::action::IntAct;
    using GoalHandle = rclcpp_action::ServerGoalHandle<IntAct>;

    explicit OmxLoadingAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("omx_loading_node", options) {
        this->server_ = rclcpp_action::create_server<IntAct>(
            this,
            "server",
            std::bind(&OmxLoadingAction::handle_goal, this, _1, _2),
            std::bind(&OmxLoadingAction::handle_cancel, this, _1),
            std::bind(&OmxLoadingAction::handle_accepted, this, _1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::Int32>("pub", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "sub", 10, std::bind(&OmxLoadingAction::receiveTopic, this, _1)
        );

        act_result_flag_ = false; 
        act_reseult_data_ = 0;
    }

private:
    rclcpp_action::Server<IntAct>::SharedPtr server_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    std::atomic<bool> act_result_flag_{false};
    int act_reseult_data_; // 토픽으로 건내받은 값 저장

    void receiveTopic(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const IntAct::Goal> goal); 
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle); 
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle); 
    void execute(const std::shared_ptr<GoalHandle> goal_handle); 
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmxLoadingAction::receiveTopic(const std_msgs::msg::Int32::SharedPtr msg) {
    act_reseult_data_ = msg->data;
    act_result_flag_ = true;
}

rclcpp_action::GoalResponse OmxLoadingAction::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const IntAct::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with ID: %d", goal->goal_num);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OmxLoadingAction::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OmxLoadingAction::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() {
        this->execute(goal_handle);
    }).detach();
}

void OmxLoadingAction::execute(const std::shared_ptr<GoalHandle> goal_handle) {
    act_result_flag_ = false;
    act_reseult_data_ = 0;
    auto result = std::make_shared<IntAct::Result>();

    // 동작 노드로 명령 전송
    auto msg = std_msgs::msg::Int32();
    msg.data = goal_handle->get_goal()->goal_num;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Executing goal: %d", msg.data);

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !act_result_flag_) {
        // 취소 명령 확인
        if (goal_handle->is_canceling()) {
            result->result_state = 1; 
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
            return;
        }
        loop_rate.sleep(); 
    }

    // Ctrl+C로 인해 노드가 종료되었을 경우
    if (!rclcpp::ok()) {
        return;
    }

    result->result_state = act_reseult_data_; 
    
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded. Result: %d", result->result_state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
 
  auto action_server = std::make_shared<OmxLoadingAction>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
 
  rclcpp::shutdown();
  return 0;
}