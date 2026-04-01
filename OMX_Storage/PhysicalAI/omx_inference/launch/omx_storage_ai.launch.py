# omx_storage_ai.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ai_inference_node = Node(
        package='omx_inference',
        executable='omx_storage_ai_node',
        name='omx_storage_ai_node',
        namespace='storage', 
        output='screen',
    )

    bridge_node = Node(
        package='omx_action',
        executable='omx_storage_action_node',
        name='omx_storage_node',
        namespace='storage',
        output='screen',
    )

    return LaunchDescription([
        ai_inference_node,
        bridge_node
    ])