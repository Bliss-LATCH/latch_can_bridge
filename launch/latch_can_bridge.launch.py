from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Single instance with default namespace
        Node(
            package='latch_can_bridge',
            executable='latch_can_bridge',
            name='latch_can_bridge',
        ),
        Node(
            package='latch_can_bridge',
            executable='hw_bridge_main',
            name='hw_bridge_main',
        ),
    ])
