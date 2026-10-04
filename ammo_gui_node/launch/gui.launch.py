"""Operator GUI plus the status bridge it depends on.

    ros2 launch ammo_gui_node gui.launch.py

The simulation is NOT started here - bring that up once in its own terminal
and leave it running:

    ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
        world:=simple_trees parrot:=true nav2:=true husky_yolo:=true \
        ground_truth_localization:=true

The mission nodes are not started here either; the GUI starts them itself
when you press Launch, so it can pause and resume them individually.
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
