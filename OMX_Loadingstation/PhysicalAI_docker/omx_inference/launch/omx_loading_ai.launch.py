# omx_loading_ai.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ai_inference_node = Node(
        package='omx_inference',
        executable='omx_loading_ai_node',
        name='omx_loading_ai_node',
        namespace='loading', 
        output='screen',
    )

    bridge_node = Node(
        package='omx_action',
        executable='omx_loading_action_node',
        name='omx_loading_node',
        namespace='loading',
        output='screen',
    )

    return LaunchDescription([
        ai_inference_node,
        bridge_node
    ])