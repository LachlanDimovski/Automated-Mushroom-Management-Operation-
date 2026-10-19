"""The Debug UI on its own.

    ros2 launch ammo_gui_node debug.launch.py
    ros2 launch ammo_gui_node debug.launch.py bridge:=true

The window is read-only, so this is safe to run at any point in a mission.
`bridge:=true` also starts a status bridge, for watching a mission that
somebody started without the operator GUI - do not use it while the GUI is
running, or two bridges will publish the same topic. Such a mission never
sends a 'start', so that bridge is told to begin its clock the first time the
rover says anything.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='false',
                              description='also run a status bridge (not with the GUI)'),
        Node(package='ammo_gui_node', executable='mission_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('bridge')),
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True}]),
        Node(package='ammo_gui_node', executable='debug', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
