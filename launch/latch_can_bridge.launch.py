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
        
        # Or with explicit namespace
        # Node(
        #     package='latch_can_bridge',
        #     executable='latch_can_bridge_node',
        #     name='latch_can_bridge',
        #     namespace='vehicle/can',  # Topics will be under vehicle/can/
        # ),
        
        # Multiple instances on different CAN buses
        # Node(
        #     package='latch_can_bridge',
        #     executable='latch_can_bridge_node',
        #     name='can_primary',
        #     namespace='vehicle/primary_can',
        #     parameters=[
        #         {'can_interface': 'can0'},
        #     ],
        # ),
        # Node(
        #     package='latch_can_bridge',
        #     executable='latch_can_bridge_node',
        #     name='can_secondary',
        #     namespace='vehicle/secondary_can',
        #     parameters=[
        #         {'can_interface': 'can1'},
        #     ],
        # ),
    ])
