import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 환경 설정
    map_path = os.path.expanduser('~/map_nowall.yaml')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')

    # 2. Navigation2 실행
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_dir),
        launch_arguments={'map': map_path}.items(),
    )

    # 3. 제어 노드 실행
    start_direct_control_node = Node(
        package='logistics_pkg',
        executable='turtlebot3_direct_control',
        name='turtlebot3_control',
        output='screen'
    )

    # 4. 초기 위치 좌표 적용 (-0.046, 0.0, 0.00642 rad)
    # yaw 0.00642 rad를 쿼터니언으로 변환 시 z는 약 0.003, w는 약 1.0입니다.
    set_initial_pose = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[[
                    'ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "',
                    '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'map\'}, ',
                    'pose: {pose: {position: {x: -0.046, y: 0.0, z: 0.0}, ',
                    'orientation: {x: 0.0, y: 0.0, z: 0.003, w: 1.0}}, ',
                    'covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}}"'
                ]],
                shell=True
            )
        ]
    )

    return LaunchDescription([
        start_navigation2,
        start_direct_control_node,
        set_initial_pose
    ])