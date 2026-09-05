"""Launches mushroom_mission_node - run instead of skidsteer_controller_node
(and instead of quad_follow_node, if that's running).

Also brings up:
  - skidsteer_controller's collision_guard_node, for the same reason
    quad_follow.launch.py does: this node publishes to husky1/cmd_vel_raw,
    not husky1/cmd_vel directly, and only collision_guard_node forwards
    that into what the Gazebo bridge actually relays into the sim.
  - skidsteer_perception's target_selector, with drive_to_target:=true, so
    husky1/target_info reflects the husky's own live camera lock rather
    than always reporting "no target" (its default when nothing else
    starts it) - that's what lets mushroom_mission_node's visual-confirm
    phase actually engage once it gets close.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    triangle_button = LaunchConfiguration('triangle_button')
    r1_button = LaunchConfiguration('r1_button')
    detection_fresh_s = LaunchConfiguration('detection_fresh_s')
    auto_trigger = LaunchConfiguration('auto_trigger')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('triangle_button', default_value='2'))
    ld.add_action(DeclareLaunchArgument('r1_button', default_value='5'))
    # How stale a YOLO detection on the quad is allowed to be for a press to
    # still count - widen this further if the feed is running slower still.
    ld.add_action(DeclareLaunchArgument('detection_fresh_s', default_value='4.0'))
    # Default True for now: dispatches on YOLO alone, no controller involved,
    # so the drive/approach/visual-confirm pipeline can be tested in
    # isolation from the R1+Triangle input path. Set to False once that's
    # confirmed working, to go back to requiring a deliberate mark.
    ld.add_action(DeclareLaunchArgument('auto_trigger', default_value='True'))

    ld.add_action(Node(
        package='ammo_mission_control',
        executable='mushroom_mission_node',
        name='mushroom_mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'triangle_button': ParameterValue(triangle_button, value_type=int),
            'r1_button': ParameterValue(r1_button, value_type=int),
            'detection_fresh_s': ParameterValue(detection_fresh_s, value_type=float),
            'auto_trigger': ParameterValue(auto_trigger, value_type=bool),
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
