#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <boost/asio.hpp>
#include <thread>

using namespace boost::asio;
using ip::tcp;

class TcpBridgeNode : public rclcpp::Node {
public:
    TcpBridgeNode() : Node("tcp_bridge_node"), socket_(io_service_) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/tcp_pub", 10, std::bind(&TcpBridgeNode::send_callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/tcp_sub", 10);

        // TCP
        tcp::endpoint endpoint(ip::address::from_string("192.168.0.6"), 8080);
        while (rclcpp::ok()) {
            try {
                RCLCPP_INFO(this->get_logger(), "Connecting...");
                socket_.connect(endpoint);
                RCLCPP_INFO(this->get_logger(), "Connected TCP Server!");
                break;
            } catch (const boost::system::system_error& e) {
                RCLCPP_WARN(this->get_logger(), "Connect Failed. Retrying in 1 second... (%s)", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        receive_thread_ = std::thread(&TcpBridgeNode::receive_loop, this);
    }
    ~TcpBridgeNode() {
        if (socket_.is_open()) socket_.close();
        if (receive_thread_.joinable()) receive_thread_.join();
    }

private:
    io_service io_service_;
    tcp::socket socket_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::thread receive_thread_;

    void send_callback(const std_msgs::msg::Int32::SharedPtr msg); // 토픽으로 받은 값을 tcp로 보냄
    void receive_loop();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TcpBridgeNode::send_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data < 0 || msg->data > 2) return;

    std::string data = std::to_string(msg->data);
    RCLCPP_INFO(this->get_logger(), "Receive Topic: %s", data.c_str());

    write(socket_, buffer(data));
    RCLCPP_INFO(this->get_logger(), "Send TCP: %s", data.c_str());
}

void TcpBridgeNode::receive_loop() {
    try {
        while (rclcpp::ok()) {
            char buf[128] = {0};
            boost::system::error_code error;
            
            // 소켓에서 데이터 읽기
            size_t len = socket_.read_some(buffer(buf), error);
            if (error == boost::asio::error::eof) {
                RCLCPP_WARN(this->get_logger(), "Disconnected Server.");
                break;
            } else if (error) {
                throw boost::system::system_error(error);
            }

            // 받은 문자열(buf)을 정수(int)로 변환
            std::string received_str(buf, len);
            int command = std::stoi(received_str);
            RCLCPP_INFO(this->get_logger(), "Receive TCP: %d", command);

            // Topic
            auto msg = std_msgs::msg::Int32();
            msg.data = command;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Send Topic: %d", msg.data);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error Thread: %s", e.what());
    }
}