"""Cage capture test: cage_capture_node alone (drives husky1/cmd_vel, no guard)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('world', default_value='simple_trees'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(Node(
        package='ammo_mission_control', executable='cage_capture_node', name='cage_capture',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'world': LaunchConfiguration('world')}]))
    ld.add_action(Node(
        package='skidsteer_perception', executable='target_selector', name='target_selector',
        output='screen', parameters=[{'use_sim_time': use_sim_time, 'drive_to_target': True}]))
    return ld
