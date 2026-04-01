# hcr3.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 네임스페이스 설정
    ns = 'hcr3'

    # 액션 서버 노드
    hcr3_action_node = Node(
        package='hcr3_action',
        executable='hcr3_action_node', 
        name='hcr3_node',
        namespace='hcr3',
        output='screen'
    )

    # 브릿지 노드
    hcr3_bridge_node = Node(
        package='hcr3_bridge_pkg',
        executable='hcr3_bridge_node', 
        name='hcr3_bridge',
        namespace='hcr3',
        output='screen'
    )

    return LaunchDescription([
        hcr3_action_node,
        hcr3_bridge_node
    ])