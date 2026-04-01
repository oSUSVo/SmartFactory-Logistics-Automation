#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <boost/asio.hpp>
#include <thread>

using namespace boost::asio;
using ip::tcp;

class TurtleTCP : public rclcpp::Node {
public: 
    TurtleTCP() 
    : Node("turtle_tcp_bridge_node"), acceptor_(io_service_, tcp::endpoint(tcp::v4(), 8080)), socket_(io_service_) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/turtle_sub", 10, std::bind(&TurtleTCP::send_callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/turtle_pub", 10);

        // TCP
        RCLCPP_INFO(this->get_logger(), "Robot Server Started. Waiting for Manager on Port 8080...");
        receive_thread_ = std::thread(&TurtleTCP::run_server, this);
    }
    ~TurtleTCP() {
        if (receive_thread_.joinable()) receive_thread_.join();
    }

private:
    io_service io_service_;
    tcp::acceptor acceptor_;
    tcp::socket socket_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::thread receive_thread_;

    void send_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void run_server();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TurtleTCP::send_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data < 1 || msg->data > 2) return;

    std::string data = std::to_string(msg->data) + "\n";
    write(socket_, buffer(data));
    RCLCPP_INFO(this->get_logger(), "Send TCP: %s", data.c_str());
}

void TurtleTCP::run_server() {
    try {
        // 대기합니다.
        acceptor_.accept(socket_); 
        RCLCPP_INFO(this->get_logger(), "Manager Connected!");

        // 연결된 후에는 데이터를 계속 받습니다.
        while (rclcpp::ok()) {
            char data[1024] = {0};
            boost::system::error_code error;
            
            size_t len = socket_.read_some(buffer(data), error); 

            if (error == error::eof) {
                RCLCPP_WARN(this->get_logger(), "Manager Disconnected.");
                break; 
            }
            // 받은 명령(숫자)을 ROS 토픽으로 발행
            auto msg = std_msgs::msg::Int32();
            msg.data = std::atoi(data);
            RCLCPP_INFO(this->get_logger(), "Receive TCP: %d", msg.data);
            
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Send Topic: %d", msg.data);
        }
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Server Error: %s", e.what());
    }
}