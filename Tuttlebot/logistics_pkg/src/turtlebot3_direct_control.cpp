// turtlebot3_direct_control.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>

class LogisticsController : public rclcpp::Node {
public:
    LogisticsController() 
        : Node("turtlebot3_control"), current_wp_index_(0),
          current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
          current_state_(State::IDLE),
          is_aligning_(false) { 
        
        initWaypoints();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/turtle_pub", 10, std::bind(&LogisticsController::commandCallback, this, std::placeholders::_1)
        );

        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/turtle_sub", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LogisticsController::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Logistics Controller (실측 좌표 적용 & 방향 제어 버전) 시작됨.");

        init_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
            setInitialPose(); 
            //RCLCPP_INFO(this->get_logger(), "✅ 초기 상태(1: 적재소 대기)를 서버에 보고합니다.");
            //sendStatus(1);
            init_timer_->cancel();
        });
    }

private:
    struct Waypoint { double x, y, yaw; };

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; 
    rclcpp::TimerBase::SharedPtr init_timer_, control_timer_;

    Waypoint wp_load_, wp_rail_; 
    std::vector<Waypoint> active_path_; 
    size_t current_wp_index_;
    double current_x_, current_y_, current_yaw_;

    enum class State { IDLE, TO_RAIL, TO_LOAD_0, TO_LOAD_1};
    State current_state_;
    bool is_aligning_; 

    void initWaypoints() {
        // ✅ [수정됨] 첫 번째 사진(적재소) 실측 데이터 적용
        // Translation: [0.094, -0.032], Yaw: 0.049 rad
        wp_load_ = {-0.046, 0.020, 0.00642}; 
        
        // ✅ [수정됨] 두 번째 사진(레일) 실측 데이터 적용
        // Translation: [0.960, -0.366], Yaw: 0.157 rad
        wp_rail_ = {1.030, -0.366, 0.157}; 
    }

    void setInitialPose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        // ✅ [수정됨] 적재소 실측 위치 세팅
        msg.pose.pose.position.x = wp_load_.x; 
        msg.pose.pose.position.y = wp_load_.y; 
        msg.pose.pose.position.z = 0.000; 

        tf2::Quaternion q;
        q.setRPY(0, 0, wp_load_.yaw); // wp_load_.yaw = 0.049

        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = -0.003;
        msg.pose.pose.orientation.w = 0.999;

        msg.pose.covariance[0] = 0.1;
        msg.pose.covariance[7] = 0.1;
        msg.pose.covariance[35] = 0.05;

        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "초기 위치 설정 완료: x=%.3f, y=%.3f, yaw=%.3f", 
                    wp_load_.x, wp_load_.y, wp_load_.yaw);
    }

    void commandCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    
        if (msg->data == 0 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 0 수신: 적재소로 이동 시작");
            current_state_ = State::TO_LOAD_0;
            active_path_ = {wp_load_}; 
            current_wp_index_ = 0;
            is_aligning_ = false;
            
        }
        else if (msg->data == 1 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 1 수신: 적재소로 이동 시작");
            current_state_ = State::TO_LOAD_1;
            active_path_ = {wp_load_}; 
            current_wp_index_ = 0;
            is_aligning_ = false;
        } 
        else if (msg->data == 2 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 2 수신: 레일로 직행");
            current_state_ = State::TO_RAIL;
            active_path_ = {wp_rail_};
            current_wp_index_ = 0;
            is_aligning_ = false;
        }
    }

    void sendStatus(int status_code) { 
        auto msg = std_msgs::msg::Int32(); 
        msg.data = status_code; 
        status_pub_->publish(msg); 
    }
    
    void stopRobot() { 
        auto twist = geometry_msgs::msg::Twist(); 
        twist.linear.x = 0.0; 
        twist.angular.z = 0.0; 
        cmd_vel_pub_->publish(twist); 
    }

    void controlLoop() {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            if (current_state_ != State::IDLE) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "로봇 위치를 찾을 수 없습니다.");
            return;
        }

        current_x_ = transformStamped.transform.translation.x;
        current_y_ = transformStamped.transform.translation.y;
        tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        if (current_state_ == State::IDLE || active_path_.empty()) return;

        Waypoint target = active_path_[current_wp_index_]; 
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        auto twist = geometry_msgs::msg::Twist();

        // [1단계: 위치 도착 판정]
        if (distance < 0.08 && !is_aligning_) { 
            RCLCPP_INFO(this->get_logger(), "최종 방향 정렬을 시작합니다.");
            stopRobot(); 
            is_aligning_ = true; 
            return;
        }

        if (is_aligning_) {
            // [2단계: 최종 방향 정렬]
            double yaw_error = target.yaw - current_yaw_;
            while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            // 오차 약 3도 이내면 정렬 완료
            if (std::abs(yaw_error) < 0.05) { 
                stopRobot();
                is_aligning_ = false;
                current_wp_index_++;

                if (current_wp_index_ >= active_path_.size()) {
                    
                    if (current_state_ == State::TO_LOAD_0) {
                        current_state_ = State::IDLE;
                        RCLCPP_INFO(this->get_logger(), "적재소 안착 및 정렬 완료! (서버에 0 전송)");
                        sendStatus(0);
                    } else if (current_state_ == State::TO_LOAD_1) {
                        current_state_ = State::IDLE;
                        RCLCPP_INFO(this->get_logger(), "적재소 안착 및 정렬 완료! (서버에 1 전송)");
                        sendStatus(1);
                    } else if (current_state_ == State::TO_RAIL) {
                        current_state_ = State::IDLE;
                        RCLCPP_INFO(this->get_logger(), "레일 안착 및 정렬 완료! (서버에 2 전송)");
                        sendStatus(2);
                    }
                }
                return;
            }

            twist.linear.x = 0.0; 
            double angular_speed = std::max(0.15, std::min(std::abs(yaw_error) * 1.0, 0.4));
            twist.angular.z = (yaw_error > 0) ? angular_speed : -angular_speed;

        } else {
            // [주행 중]
            double target_yaw = std::atan2(dy, dx);
            double diff_yaw = target_yaw - current_yaw_;
            while (diff_yaw > M_PI) diff_yaw -= 2.0 * M_PI;
            while (diff_yaw < -M_PI) diff_yaw += 2.0 * M_PI;

            if (std::abs(diff_yaw) > 0.15) { 
                twist.linear.x = 0.0;
                double angular_speed = std::max(0.2, std::min(std::abs(diff_yaw) * 0.8, 0.5));
                twist.angular.z = (diff_yaw > 0) ? angular_speed : -angular_speed;
            } else {
                double linear_speed = distance * 0.8; 
                twist.linear.x = std::max(0.07, std::min(linear_speed, 0.15)); 
                twist.angular.z = diff_yaw * 0.8; 
            }
        }

        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<LogisticsController>()); 
    rclcpp::shutdown(); 
    return 0; 
}