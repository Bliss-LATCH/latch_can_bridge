from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Single instance with default namespace
        Node(
            package='latch_can_bridge',
            executable='latch_can_bridge_node',
            name='latch_can_bridge',
        ),
        Node(
            package='front_tred_hw_bridge',
            executable='front_tred_hw_bridge',
            name='front_tred_hw_bridge',
        ),
    ])
