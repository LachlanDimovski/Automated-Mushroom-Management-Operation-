"""Launch the quadcopter state-machine / mapping display.

  ros2 launch ~/quad_showcase/quad_showcase.launch.py                  # display only (mission already running, e.g. started from the GUI)
  ros2 launch ~/quad_showcase/quad_showcase.launch.py mission:=true    # also start cage_mission.launch.py
  ros2 launch ~/quad_showcase/quad_showcase.launch.py demo:=true       # synthetic data, no simulator needed

The display only subscribes; it publishes nothing and cannot disturb the mission.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCRIPT = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'quad_showcase.py')
ENV = {'QT_QPA_PLATFORM': 'xcb', 'LIBGL_ALWAYS_SOFTWARE': '1'}


def generate_launch_description():
    mission_launch = os.path.join(get_package_share_directory('ammo_mission_control'), 'launch', 'cage_mission.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='false', description='also start cage_mission.launch.py'),
        DeclareLaunchArgument('world', default_value='simple_trees', description='world for mission:=true'),
        DeclareLaunchArgument('headless', default_value='true', description='no Gazebo window for mission:=true'),
        DeclareLaunchArgument('demo', default_value='false', description='synthetic data, no ROS topics needed'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={'world': LaunchConfiguration('world'), 'headless': LaunchConfiguration('headless')}.items(),
            condition=IfCondition(LaunchConfiguration('mission'))),
        ExecuteProcess(cmd=['python3', SCRIPT], name='quad_showcase', output='screen', additional_env=ENV,
                       condition=UnlessCondition(LaunchConfiguration('demo'))),
        ExecuteProcess(cmd=['python3', SCRIPT, '--demo'], name='quad_showcase_demo', output='screen', additional_env=ENV,
                       condition=IfCondition(LaunchConfiguration('demo'))),
    ])
