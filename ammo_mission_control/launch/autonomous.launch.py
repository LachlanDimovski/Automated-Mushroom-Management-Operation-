"""Fully autonomous search-and-rescue - no controller involved anywhere.

Bundles everything that used to need a PS4 controller (ps4_drone_test.py)
plus the mushroom mission stack into one launch:

  - quad_search_node: autonomous lawnmower search flight (replaces
    ps4_drone_test.py entirely)
  - mushroom_mission_node: auto_trigger on by default - dispatches the
    skidsteer the instant the quad's YOLO reacquires a detection, no
    button needed
  - collision_guard_node + target_selector: same reasons as
    mushroom_mission.launch.py

    ros2 launch ammo_mission_control autonomous.launch.py

Run instead of: ps4_drone_test.py, skidsteer_controller_node, quad_follow_node,
and mushroom_mission.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    detection_fresh_s = LaunchConfiguration('detection_fresh_s')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('detection_fresh_s', default_value='4.0'))

    ld.add_action(Node(
        package='ammo_mission_control',
        executable='quad_search_node',
        name='quad_search_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    ld.add_action(Node(
        package='ammo_mission_control',
        executable='mushroom_mission_node',
        name='mushroom_mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'detection_fresh_s': ParameterValue(detection_fresh_s, value_type=float),
            'auto_trigger': True,
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

    ld.add_action(Node(
        package='skidsteer_perception',
        executable='target_selector',
        name='target_selector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'drive_to_target': True}],
    ))

    return ld
