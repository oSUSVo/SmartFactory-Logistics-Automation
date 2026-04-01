#include "rclcpp/rclcpp.hpp"
//#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <boost/asio.hpp>
#include <thread>

using namespace boost::asio;
using ip::tcp;

class TurtleTCP : public rclcpp::Node {
public: 
    TurtleTCP() 
    : Node("tcp_bridge_node"), acceptor_(io_service_, tcp::endpoint(tcp::v4(), 8080)), socket_(io_service_) {
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

    std::string data = std::to_string(msg->data);
    write(socket_, buffer(data));
    RCLCPP_INFO(this->get_logger(), "Send Massage by TCP: %s", data.c_str());
}

void TurtleTCP::run_server() {
    try {
        acceptor_.accept(socket_); 
        RCLCPP_INFO(this->get_logger(), "Manager Connected!");

        while (rclcpp::ok()) {
            char data[1024] = {0}; // 루프마다 초기화
            boost::system::error_code error;
            
            // 1. 실제로 받은 길이를 확인합니다 (중요!)
            size_t len = socket_.read_some(buffer(data), error);
            if (len > 0) {
                // 1024바이트 중 딱 받은 만큼만 null 문자로 닫아줍니다.
                data[len] = '\0'; 
                
                // 실제로 뭐라고 들어오는지 정확히 확인!
                RCLCPP_INFO(this->get_logger(), "Raw TCP data: [%s] (length: %ld)", data, len);

                auto msg = std_msgs::msg::Int32();
                msg.data = std::atoi(data);
                publisher_->publish(msg);
            }

            if (error == error::eof) {
                RCLCPP_WARN(this->get_logger(), "Manager Disconnected.");
                break; 
            }

            if (len > 0) {
                // 2. 받은 길이만큼만 잘라서 문자열로 만듭니다.
                std::string received_data(data, len);
                
                try {
                    auto msg = std_msgs::msg::Int32();
                    // 3. stoi를 써서 안전하게 변환 (숫자가 아닐 경우 예외처리 가능)
                    msg.data = std::stoi(received_data);
                    
                    publisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg.data);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid data received: %s", received_data.c_str());
                }
            }
        }
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Server Error: %s", e.what());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto tcp_bridge_node = std::make_shared<TurtleTCP>();
    rclcpp::spin(tcp_bridge_node); 

    rclcpp::shutdown();

    return 0;
}
