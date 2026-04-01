#include "smart_factory_server/server/hcr3.hpp"
#include "smart_factory_server/server/omx_loading.hpp"
#include "smart_factory_server/server/omx_storage.hpp"
#include "smart_factory_server/server/turtle_tcp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
 
  auto hcr3 = std::make_shared<Hcr3Action>();
  auto omx_loading = std::make_shared<OmxLoadingAction>();
  auto omx_storage = std::make_shared<OmxStorageAction>();
  auto turtle_tcp = std::make_shared<TurtleTCP>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(hcr3);
  executor.add_node(omx_loading);
  executor.add_node(omx_storage);
  executor.add_node(turtle_tcp);
  executor.spin();
 
  rclcpp::shutdown();
  return 0;
}