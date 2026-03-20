# omx_storage_inference.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omx_inference',
            executable='omx_storage_ai_node',
            name='omx_storage_ai_node',
            output='screen',
            parameters=[{
                'mode':         'storage',   # /omx_storage_pub → /omx_storage_sub
            }]
        )
    ])