"""The Fleet Debug UI with its status bridge, for a fleet_mission run.

    ros2 launch ammo_gui_node fleet_debug.launch.py

The timeline and state cards read ammo/mission/status and ammo/mission/events,
which only mission_bridge publishes. Do not run this alongside the operator GUI,
which starts its own bridge.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # autostart: a fleet run has no operator window to press Launch, so the bridge starts
        # the mission on the rover's first log line
        Node(package='ammo_gui_node', executable='mission_bridge', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True}]),
        Node(package='ammo_gui_node', executable='fleet_debug', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
