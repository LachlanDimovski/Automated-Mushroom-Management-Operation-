"""Launches quad_follow_node - run instead of skidsteer_controller_node.

Also brings up skidsteer_controller's collision_guard_node, since
quad_follow_node publishes to husky1/cmd_vel_raw (not husky1/cmd_vel
directly) - the Gazebo bridge only relays husky1/cmd_vel into the sim, so
without collision_guard_node forwarding raw -> gated cmd_vel, the husky
never actually receives any drive command at all.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    delay_s = LaunchConfiguration('delay_s')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('delay_s', default_value='10.0'))

    ld.add_action(Node(
        package='ammo_mission_control',
        executable='quad_follow_node',
        name='quad_follow_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'delay_s': ParameterValue(delay_s, value_type=float),
            'husky_spawn': [0.0, 0.0],
            'parrot_spawn': [2.0, 0.0],
        }],
    ))

    ld.add_action(Node(
        package='skidsteer_controller',
        executable='collision_guard_node',
        name='collision_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    return ld
