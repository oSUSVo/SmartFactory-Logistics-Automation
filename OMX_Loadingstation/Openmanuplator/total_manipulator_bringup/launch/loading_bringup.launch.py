import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # 1. 경로 설정
    pkg_path = get_package_share_directory('open_manipulator_bringup') 
    camera_launch_path = os.path.join(pkg_path, 'launch', 'camera_usb_cam.launch.py')
    follower_path = os.path.join(pkg_path, 'launch', 'omx_f_follower_ai.launch.py')

    # 2. 인자(Argument) 선언
    declare_init_position = DeclareLaunchArgument(
        'init_position',
        default_value='false',
        description='Whether to initialize position'
    )
    
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to start rviz'
    )

    # 3. [카메라 1] 정의 (Namespace: loading)
    camera1_group = GroupAction([
        PushRosNamespace('loading'), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            launch_arguments={'name': 'camera1', 'video_device': '/dev/video0'}.items()
        )
    ])
    
    # 4. [카메라 2] 정의 (Namespace: loading)
    camera2_group = GroupAction([
        PushRosNamespace('loading'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            launch_arguments={'name': 'camera2', 'video_device': '/dev/video2'}.items()
        )
    ])

    # 5. [팔로워] 정의 (Namespace: loading 추가됨)
    # 기존에는 PushRosNamespace 밖에 있어서 토픽이 꼬였던 부분을 수정했습니다.
    follower_group = GroupAction([
        PushRosNamespace('loading'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follower_path),
            launch_arguments={
                'start_rviz': LaunchConfiguration('start_rviz'), 
                'init_position': LaunchConfiguration('init_position')
            }.items()
        )
    ])

    # --- 실행 구성 ---

    return LaunchDescription([
        # 인자 선언
        declare_init_position,
        declare_start_rviz,

        # 1. 즉시 실행: 카메라 1
        camera1_group,

        # 2. 5초 대기 후 실행: 카메라 2
        TimerAction(
            period=5.0,
            actions=[camera2_group]
        ),

        # 3. 10초 대기 후 실행: 팔로워 (이제 /loading/joint_states 등으로 발행됩니다)
        TimerAction(
            period=10.0,
            actions=[follower_group]
        )
    ])