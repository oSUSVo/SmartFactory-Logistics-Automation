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
          is_parking_(false),
          current_state_(State::IDLE) {
        
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

        RCLCPP_INFO(this->get_logger(), "Logistics Controller (새로운 좌표 & 10초 대기 버전) 시작됨.");

        // 🌟 Nav2와 AMCL이 완전히 켜질 때까지 10초 기다렸다가 초기 위치를 쏩니다!
        init_timer_ = this->create_wall_timer(std::chrono::seconds(10), [this]() {
            setInitialPose(); 
            RCLCPP_INFO(this->get_logger(), "✅ 초기 상태(1: 적재소 대기)를 서버에 보고합니다.");
            sendStatus(1);
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
    
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    Waypoint wp_load_, wp_rail_, wp1_, wp2_, wp3_;
    std::vector<Waypoint> active_path_; 
    size_t current_wp_index_;        

    double current_x_, current_y_, current_yaw_;
    bool is_parking_; 

    enum class State { IDLE, TO_RAIL, TO_LOAD };
    State current_state_;

    void initWaypoints() {
        // 🌟 사용자 매핑 데이터 기반 갱신 완료!

        // [첫 번째 사진] 적재소(시작점) 좌표 갱신
        wp_load_ = {0.032, 0.037, 0.002}; 
        
        // [두 번째 사진] 레일 좌표 갱신
        wp_rail_ = {0.882, -0.296, -0.008}; 

        // 다른 경유지들은 기존 값 유지
        wp1_     = {0.436, 0.713, -1.549};
        wp2_     = {0.464, 0.154, -1.532};
        wp3_     = {0.398, -0.008, -3.022};
    }

    void setInitialPose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = wp_load_.x;
        msg.pose.pose.position.y = wp_load_.y;
        msg.pose.pose.position.z = 0.010;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wp_load_.yaw);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.068;

        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "📍 로봇의 초기 위치를 적재소(x:%.3f, y:%.3f)로 설정했습니다.", wp_load_.x, wp_load_.y);
    }

    void commandCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 1 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 1 수신: 적재소로 이동");
            current_state_ = State::TO_LOAD;
            active_path_ = {wp_load_};
            current_wp_index_ = 0;
            is_parking_ = false; 
        } 
        else if (msg->data == 2 && current_state_ == State::IDLE) {
            RCLCPP_INFO(this->get_logger(), "명령 2 수신: 레일로 직행");
            current_state_ = State::TO_RAIL;
            active_path_ = {wp_rail_};
            current_wp_index_ = 0;
            is_parking_ = false; 
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
            if (current_state_ != State::IDLE) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "⚠️ 로봇 절대 위치 획득 실패. 지도를 확인하세요!");
            }
            return;
        }

        current_x_ = transformStamped.transform.translation.x;
        current_y_ = transformStamped.transform.translation.y;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        if (current_state_ == State::IDLE || active_path_.empty()) return;

        Waypoint target = active_path_[current_wp_index_];
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_yaw = std::atan2(dy, dx);

        double diff_yaw = target_yaw - current_yaw_;
        while (diff_yaw > M_PI) diff_yaw -= 2.0 * M_PI;
        while (diff_yaw < -M_PI) diff_yaw += 2.0 * M_PI;

        auto twist = geometry_msgs::msg::Twist();
        bool is_final_destination = (current_wp_index_ == active_path_.size() - 1);

        if (is_final_destination) {
            if (!is_parking_ && distance <= 0.02) { 
                is_parking_ = true;
                RCLCPP_INFO(this->get_logger(), "목표 2cm 이내 진입! 제자리 회전을 시작합니다.");
            } else if (is_parking_ && distance > 0.05) { 
                is_parking_ = false;
                RCLCPP_INFO(this->get_logger(), "회전 중 목표에서 멀어졌습니다. 다시 다가갑니다.");
            }
        } else {
            if (distance < 0.05) {
                stopRobot();
                current_wp_index_++; 
                RCLCPP_INFO(this->get_logger(), "경유지 통과! 다음으로 이동 중...");
                return;
            }
        }

        if (is_parking_) {
            double final_diff_yaw = target.yaw - current_yaw_;
            while (final_diff_yaw > M_PI) final_diff_yaw -= 2.0 * M_PI;
            while (final_diff_yaw < -M_PI) final_diff_yaw += 2.0 * M_PI;

            if (std::abs(final_diff_yaw) > 0.08) {
                twist.linear.x = 0.0; 
                twist.angular.z = (final_diff_yaw > 0) ? 0.25 : -0.25; 
                cmd_vel_pub_->publish(twist);
                return;
            } else {
                stopRobot();
                is_parking_ = false; 

                if (current_state_ == State::TO_LOAD) {
                    current_state_ = State::IDLE;
                    RCLCPP_INFO(this->get_logger(), "✅ 적재소 정밀 주차 완료 (서버에 1 전송)");
                    sendStatus(1);
                } else if (current_state_ == State::TO_RAIL) {
                    current_state_ = State::IDLE;
                    RCLCPP_INFO(this->get_logger(), "✅ 레일 정밀 주차 완료 (서버에 2 전송)");
                    sendStatus(2);
                }
                return;
            }
        }

        if (std::abs(diff_yaw) > 0.15) { 
            twist.linear.x = 0.0; 
            twist.angular.z = (diff_yaw > 0) ? 0.35 : -0.35; 
        } else {
            double speed = distance * 1.5; 
            twist.linear.x = std::min(speed, 0.15); 
            twist.angular.z = diff_yaw * 1.5;       
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
