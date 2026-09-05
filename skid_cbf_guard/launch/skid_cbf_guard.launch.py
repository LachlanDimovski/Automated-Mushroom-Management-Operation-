"""Launches cbf_guard_node - run instead of collision_guard_node to compare.

Note: cbf_guard_node currently hardcodes its tuning constants (ALPHA,
ROBOT_RADIUS_M, etc.) rather than reading ROS params - this launch file
exists for consistency with the rest of the workspace's run pattern, not
because there's anything to configure yet.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='skid_cbf_guard',
        executable='cbf_guard_node',
        name='cbf_guard',
        output='screen',
    ))

    return ld
