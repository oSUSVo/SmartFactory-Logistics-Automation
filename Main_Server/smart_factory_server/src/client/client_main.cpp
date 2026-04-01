#include "smart_factory_server/client/factory_client.hpp"
#include "smart_factory_server/client/tcp_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto factory_client = std::make_shared<factoryClient>();
    auto tcp_bridge = std::make_shared<TcpBridgeNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(factory_client);
    executor.add_node(tcp_bridge);
    executor.spin();
    
    factory_client->cancel_action();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    rclcpp::shutdown();

    return 0;
}