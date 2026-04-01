cmake_minimum_required(VERSION 3.8)
project(logistics_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)    # ✅ 추가: 아래에서 사용 중이므로 필수
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# --- Executables ---

add_executable(turtlebot3_tcp src/turtlebot3_tcp.cpp)
ament_target_dependencies(turtlebot3_tcp rclcpp std_msgs)
target_link_libraries(turtlebot3_tcp ${Boost_LIBRARIES})

add_executable(turtlebot3_control src/turtlebot3_control.cpp)
ament_target_dependencies(turtlebot3_control
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
)

add_executable(logistics_controller_node src/logistics_controller.cpp)
ament_target_dependencies(logistics_controller_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

add_executable(turtlebot3_direct_control src/turtlebot3_direct_control.cpp)
ament_target_dependencies(turtlebot3_direct_control
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(waypoint_tester_node src/waypoint_tester.cpp)
ament_target_dependencies(waypoint_tester_node rclcpp rclcpp_action nav2_msgs geometry_msgs)

# --- Install Section ---

# 노드(실행파일) 설치
install(TARGETS
  logistics_controller_node
  turtlebot3_tcp
  turtlebot3_control
  turtlebot3_direct_control
  waypoint_tester_node
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ 중요: launch 폴더 설치 구문 추가
# 이 부분이 있어야 'ros2 launch logistics_pkg ...' 명령어가 작동합니다.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()