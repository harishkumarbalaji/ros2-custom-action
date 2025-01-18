"""
Launch file for the mission action server node.

This launch file will launch the mission action server node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch the mission action server node
    talker_node = Node(
        package="mission_processor",
        executable="mission_action_server",
    )

    ld.add_action(talker_node)

    return ld
