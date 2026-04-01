cmake_minimum_required(VERSION 3.8)
project(smart_factory_server)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# build
add_executable(client_node src/client/client_main.cpp)
ament_target_dependencies(client_node rclcpp rclcpp_action custom_interfaces std_msgs)

add_executable(server_node src/server/server_main.cpp)
ament_target_dependencies(server_node rclcpp rclcpp_action custom_interfaces std_msgs)
target_link_libraries(server_node ${Boost_LIBRARIES})

install(TARGETS
  client_node
  server_node

  DESTINATION lib/${PROJECT_NAME}
 )
 
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()