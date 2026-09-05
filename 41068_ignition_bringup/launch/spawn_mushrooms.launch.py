"""Standalone mushroom (+ optional decoy) scatter for an already-running sim.

Spawns mushroom_amanita models (and optionally decoy_box/decoy_cylinder
clutter, to give the quad's YOLO detector things to correctly ignore) into
whatever Gazebo world 41068_ignition.launch.py already has up. Run this in
its own terminal AFTER the sim is running:

    ros2 launch 41068_ignition_bringup spawn_mushrooms.launch.py
    ros2 launch 41068_ignition_bringup spawn_mushrooms.launch.py count:=10 decoys:=False

Poses are randomised once per invocation, scattered SCATTER_RADIUS_MIN_M to
SCATTER_RADIUS_MAX_M from the origin (clear of both robots' spawn points) -
re-running this adds another fresh batch, it doesn't clear what's already
spawned. Standalone/new file so 41068_ignition.launch.py stays untouched;
mirrors the scatter approach already used in skidsteer_test.launch.py.
"""
import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SCATTER_RADIUS_MIN_M = 3.0
SCATTER_RADIUS_MAX_M = 9.0


def _spawn_nodes(model_name, count, spawn_z, use_sim_time, tag):
    nodes = []
    for i in range(count):
        angle = random.uniform(0.0, 2.0 * math.pi)
        radius = random.uniform(SCATTER_RADIUS_MIN_M, SCATTER_RADIUS_MAX_M)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        model_file = PathJoinSubstitution([
            FindPackageShare('41068_ignition_bringup'),
            'models', model_name, 'model.sdf',
        ])
        nodes.append(Node(
            package='ros_ign_gazebo',
            executable='create',
            name=f'spawn_{model_name}_{tag}_{i}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-file', model_file,
                '-name', f'{model_name}_{tag}_{i}',
                '-x', str(x), '-y', str(y), '-z', str(spawn_z),
            ],
        ))
    return nodes


def _make_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    count = int(LaunchConfiguration('count').perform(context))
    decoys = LaunchConfiguration('decoys').perform(context).lower() in ('true', '1', 'yes')

    nodes = _spawn_nodes('mushroom_amanita', count, 0.0, use_sim_time, 'm')
    if decoys:
        nodes += _spawn_nodes('decoy_box', max(1, count // 2), 0.3, use_sim_time, 'b')
        nodes += _spawn_nodes('decoy_cylinder', max(1, count // 2), 0.3, use_sim_time, 'c')
    return nodes


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument(
        'count', default_value='6', description='How many mushrooms to scatter'))
    ld.add_action(DeclareLaunchArgument(
        'decoys', default_value='True',
        description='Also scatter non-target decoy clutter (boxes/cylinders)'))

    ld.add_action(OpaqueFunction(function=_make_nodes))

    return ld
