#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <thread>
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/int_act.hpp"
using namespace std::placeholders;

class factoryClient : public rclcpp::Node {
public:
    using IntAct = custom_interfaces::action::IntAct;
    using GoalHandle = rclcpp_action::ClientGoalHandle<IntAct>;

    explicit factoryClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("factory_client", node_options) {
        omx_loading_ = rclcpp_action::create_client<IntAct>(this, "/loading/server");
        omx_storage_ = rclcpp_action::create_client<IntAct>(this, "/storage/server");
        hcr3_ = rclcpp_action::create_client<IntAct>(this, "/hcr3/server");

        tcp_pub_ = this->create_publisher<std_msgs::msg::Int32>("/tcp_pub", 10);
        tcp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/tcp_sub", 10,
            std::bind(&factoryClient::receiveTCP, this, std::placeholders::_1)
        );
        requset_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/request_sub", 10,
            std::bind(&factoryClient::request, this, std::placeholders::_1)
        );

    }
    void cancel_action();

private:
    rclcpp_action::Client<IntAct>::SharedPtr omx_loading_; // omx 적재소
    rclcpp_action::Client<IntAct>::SharedPtr omx_storage_; // omx 창고
    rclcpp_action::Client<IntAct>::SharedPtr hcr3_; // 한화 협동로봇

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr requset_sub; // 서브스크립션 (받는 거)
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tcp_pub_; // 퍼블리셔 (주는 거)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tcp_sub_; // 서브스크립션 (받는 거)

    GoalHandle::SharedPtr active_goal_handle_;
    std::string current_running_client_;
    
    void sendTCP(int msg);
    void receiveTCP(const std_msgs::msg::Int32::SharedPtr msg);
    void request(const std_msgs::msg::Int32::SharedPtr msg);

    void goal_response_callback(const GoalHandle::SharedPtr & goal_handle, std::string action_name);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const IntAct::Feedback> feedback);

    void omx_loading_callback(const GoalHandle::WrappedResult & result);
    void omx_storage_callback(const GoalHandle::WrappedResult & result);
    void hcr3_callback(const GoalHandle::WrappedResult & result);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void factoryClient::sendTCP(int msg) {
    auto pub = std_msgs::msg::Int32();
    pub.data = msg;
    tcp_pub_->publish(pub);
}

void factoryClient::receiveTCP(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data < 1) return;
    RCLCPP_INFO(this->get_logger(), "Receive Topic: %d", msg->data);

    auto action_option = rclcpp_action::Client<IntAct>::SendGoalOptions();
    action_option.feedback_callback = std::bind(&factoryClient::feedback_callback, this, _1, _2);

    auto goal_msg = IntAct::Goal();
    goal_msg.goal_num = 1;
    if (msg->data == 1) {
        action_option.goal_response_callback = std::bind(&factoryClient::goal_response_callback, this, _1, "loading");
        action_option.result_callback = std::bind(&factoryClient::omx_loading_callback, this, _1);
        omx_loading_->async_send_goal(goal_msg, action_option);
    }
    else if (msg->data == 2) {
        action_option.goal_response_callback = std::bind(&factoryClient::goal_response_callback, this, _1, "storage");
        action_option.result_callback = std::bind(&factoryClient::omx_storage_callback, this, _1);
        omx_storage_->async_send_goal(goal_msg, action_option);
    }
    RCLCPP_INFO(this->get_logger(), "Start OMX goal_num: %d", goal_msg.goal_num);
}

void factoryClient::request(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 1) {
        RCLCPP_INFO(this->get_logger(), "Request Box");
        sendTCP(1);
    }
}

void factoryClient::goal_response_callback(const GoalHandle::SharedPtr & goal_handle, std::string action_name) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->current_running_client_ = "";
    } else {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "%s: Goal rejected", action_name.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "%s: Goal accepted", action_name.c_str());
            this->active_goal_handle_ = goal_handle;
            this->current_running_client_ = action_name;
        }
    }
}

void factoryClient::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const IntAct::Feedback> feedback) {
    (void)feedback;
    RCLCPP_INFO(this->get_logger(), "feedback_callback");
}

void factoryClient::cancel_action() {
    if (active_goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "Canceling: %s", current_running_client_.c_str());

        if (current_running_client_ == "hcr3") hcr3_->async_cancel_goal(active_goal_handle_);
        else if (current_running_client_ == "loading") omx_loading_->async_cancel_goal(active_goal_handle_);
        else if (current_running_client_ == "storage") omx_storage_->async_cancel_goal(active_goal_handle_);
        
        active_goal_handle_ = nullptr;
        current_running_client_ = "";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void factoryClient::hcr3_callback(const GoalHandle::WrappedResult & result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "hcr3 Action Fail");
        return;
    }
    active_goal_handle_ = nullptr; // 완료되었으니 핸들 초기화
    RCLCPP_ERROR(this->get_logger(), "hcr3 Action Finish");
}

void factoryClient::omx_loading_callback(const GoalHandle::WrappedResult &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "omx_loading Action Fail");
        return;
    }
    active_goal_handle_ = nullptr; // 완료되었으니 핸들 초기화
    RCLCPP_INFO(this->get_logger(), "omx_loading Action Finish");
    sendTCP(2);
}

void factoryClient::omx_storage_callback(const GoalHandle::WrappedResult &result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "omx_storage Action Fail");
        return;
    }
    active_goal_handle_ = nullptr; // 완료되었으니 핸들 초기화
    RCLCPP_INFO(this->get_logger(), "omx_storage Action Finish");
    sendTCP(0); // 터틀봇 적재소로 돌려보냄

    auto action_option = rclcpp_action::Client<IntAct>::SendGoalOptions();
    action_option.goal_response_callback = std::bind(&factoryClient::goal_response_callback, this, _1, "hcr3");
    action_option.feedback_callback = std::bind(&factoryClient::feedback_callback, this, _1, _2);
    action_option.result_callback = std::bind(&factoryClient::hcr3_callback, this, _1);
    
    auto goal_msg = IntAct::Goal();
    goal_msg.goal_num = 1;
    hcr3_->async_send_goal(goal_msg, action_option);

    RCLCPP_INFO(this->get_logger(), "Start hcr3");
}