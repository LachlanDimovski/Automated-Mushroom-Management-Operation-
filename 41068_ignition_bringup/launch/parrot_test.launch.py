"""Standalone test launch for the Parrot drone.

Aerial counterpart to skidsteer_test.launch.py. Brings up the parrot1
platform, scatters mushrooms and decoys to look at, runs the YOLO detector on
the drone camera, and opens a view of the detections.

    ros2 launch 41068_ignition_bringup parrot_test.launch.py
    ros2 launch 41068_ignition_bringup parrot_test.launch.py hazard:=True
    ros2 launch 41068_ignition_bringup parrot_test.launch.py mushrooms:=12

The drone has no flight controller here - nothing holds altitude on its own.
Fly it from the xterm teleop window that opens:

    t / b     up / down          i / ,     forward / back
    j / l     yaw left / right   k         stop
    q / z     faster / slower

Start with t to climb, and expect to keep nudging it: with no controller the
drone holds whatever velocity it was last given.
"""
import math
import os
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Targets and clutter: (model, count, spawn height). Spawn height depends on
# where each model's origin sits - the decoy primitives are centred on their
# own origin so they need lifting clear of the ground, while the mushroom and
# person meshes have their base baked to z=0 and spawn flush.
SCATTER = [('mushroom_amanita', 8, 0.0),
           ('decoy_box', 3, 0.3),
           ('decoy_cylinder', 3, 0.3)]
RADIUS_MIN_M, RADIUS_MAX_M = 3.0, 12.0
SPAWN_DELAY_S = 10.0        # after the drone itself is up


def scatter_nodes(context, use_sim_time):
    n_mush = int(LaunchConfiguration('mushrooms').perform(context))
    nodes = []
    for model, count, z in SCATTER:
        for i in range(n_mush if model == 'mushroom_amanita' else count):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(RADIUS_MIN_M, RADIUS_MAX_M)
            nodes.append(Node(
                package='ros_ign_gazebo',
                executable='create',
                name=f'spawn_{model}_{i}',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-file', PathJoinSubstitution([
                        FindPackageShare('41068_ignition_bringup'),
                        'models', model, 'model.sdf']),
                    '-name', f'{model}_{i}',
                    '-x', str(radius * math.cos(angle)),
                    '-y', str(radius * math.sin(angle)),
                    '-z', str(z),
                    '-Y', str(random.uniform(-math.pi, math.pi)),
                ],
            ))
    return nodes


def generate_launch_description():
    # VS Code's snap sandbox injects GTK/locale vars that shadow system libs
    # with an incompatible libpthread, crashing any Qt/GTK app launched from
    # an integrated terminal. Strip them for children.
    for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
                'GIO_MODULE_DIR', 'LOCPATH', 'GSETTINGS_SCHEMA_DIR'):
        os.environ.pop(var, None)

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')

    for name, default, desc in (
        ('use_sim_time', 'True', 'Use simulation clock'),
        ('world', 'simple_trees', 'Which world to load'),
        ('headless', 'False', 'Run Gazebo without its GUI window'),
        ('yolo', 'True', 'Run the mushroom detector on parrot1/detections'),
        ('hazard', 'False', 'Also run the person detector on parrot1/hazards'),
        ('teleop', 'True', 'Open an xterm to fly the drone by keyboard'),
        ('detection_view', 'True', 'Pop up a view of parrot1/detections/image'),
        ('mushrooms', '8', 'How many mushrooms to scatter'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default,
                                            description=desc))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('41068_ignition_bringup'),
            'launch', '41068_ignition.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'husky': 'False',
            'parrot': 'True',
            'rviz': 'True',
            # The drone carries no 2D lidar mapping setup worth running here,
            # and SLAM/Nav2 would only add startup time to a perception test.
            'slam': 'False',
            'nav2': 'False',
            'yolo': LaunchConfiguration('yolo'),
            'hazard': LaunchConfiguration('hazard'),
        }.items(),
    ))

    # Keyboard teleop needs a real terminal for stdin, so give it its own.
    ld.add_action(TimerAction(period=8.0, actions=[ExecuteProcess(
        cmd=['xterm', '-title', 'parrot1 teleop', '-geometry', '80x28', '-e',
             'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
             '--ros-args', '-r', 'cmd_vel:=/parrot1/cmd_vel',
             '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )]))

    ld.add_action(TimerAction(period=12.0, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='detection_view',
        output='screen',
        arguments=['/parrot1/detections/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('detection_view')),
    )]))

    from launch.actions import OpaqueFunction
    ld.add_action(TimerAction(period=SPAWN_DELAY_S, actions=[
        OpaqueFunction(function=lambda ctx: scatter_nodes(ctx, use_sim_time))]))

    return ld
