import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, TimerAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_path = get_package_share_directory('open_manipulator_bringup') 
    camera_launch_path = os.path.join(pkg_path, 'launch', 'camera_usb_cam.launch.py')
    omx_f_follow_launch_path = os.path.join(pkg_path, 'launch', 'omx_f_follower_ai.launch.py')

    init_pos_arg = DeclareLaunchArgument(
        'init_position',
        default_value='false',
        description='Whether to launch the init_position node'
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to execute rviz2'
    )

    camera1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={'name': 'camera1', 'video_device': '/dev/video0'}.items()
    )

    camera2_delayed = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_path),
                launch_arguments={'name': 'camera2', 'video_device': '/dev/video2'}.items()
            )
        ]
    )

    storage_group = GroupAction(
        actions=[
            PushRosNamespace('storage'),
            camera1,
            camera2_delayed, 
            
            # omx_f_follow
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(omx_f_follow_launch_path),
                launch_arguments={
                    'start_rviz': 'false',
                    'init_position': 'false'
                }.items()
            ),
        ]
    )

    return LaunchDescription([init_pos_arg, start_rviz_arg, storage_group])