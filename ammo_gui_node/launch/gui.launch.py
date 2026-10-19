"""Operator GUI plus the status bridge it depends on.

    ros2 launch ammo_gui_node gui.launch.py

The simulation and mission are started from the GUI's Launch button.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='ammo_gui_node', executable='mission_bridge', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='ammo_gui_node', executable='gui', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
