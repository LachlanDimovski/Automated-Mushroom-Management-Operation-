"""Scatter people into an already-running sim, as hazards for the Husky.

Counterpart to spawn_mushrooms.launch.py. Mushrooms are targets to drive to;
people are the opposite - things to notice and not drive into. Run it in its
own terminal AFTER the sim is up:

    ros2 launch 41068_ignition_bringup spawn_people.launch.py
    ros2 launch 41068_ignition_bringup spawn_people.launch.py count:=5
    ros2 launch 41068_ignition_bringup spawn_people.launch.py radius_min:=2.0 radius_max:=6.0

Poses are randomised once per invocation. Re-running adds another batch rather
than replacing the last one.

Headings are random because a person looks quite different from the front,
side and back, and a hazard detector that only works on one aspect is not much
of a hazard detector.
"""
import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MODEL = 'walking_person'


def _make_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    count = int(LaunchConfiguration('count').perform(context))
    r_min = float(LaunchConfiguration('radius_min').perform(context))
    r_max = float(LaunchConfiguration('radius_max').perform(context))

    model_file = PathJoinSubstitution([
        FindPackageShare('41068_ignition_bringup'),
        'models', MODEL, 'model.sdf',
    ])

    nodes = []
    for i in range(count):
        angle = random.uniform(0.0, 2.0 * math.pi)
        radius = random.uniform(r_min, r_max)
        nodes.append(Node(
            package='ros_ign_gazebo',
            executable='create',
            name=f'spawn_person_{i}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-file', model_file,
                '-name', f'{MODEL}_{i}',
                '-x', str(radius * math.cos(angle)),
                '-y', str(radius * math.sin(angle)),
                # The mesh has its feet at z=0, so it stands flush.
                '-z', '0.0',
                '-Y', str(random.uniform(-math.pi, math.pi)),
            ],
        ))
    return nodes


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument(
        'count', default_value='3', description='How many people to scatter'))
    # Wider than the mushroom scatter: a person is worth detecting well before
    # the robot is on top of them, and stays above the detector's size floor
    # out to roughly 14 m.
    ld.add_action(DeclareLaunchArgument(
        'radius_min', default_value='4.0',
        description='Closest a person is placed to the origin, in metres'))
    ld.add_action(DeclareLaunchArgument(
        'radius_max', default_value='12.0',
        description='Furthest a person is placed from the origin, in metres'))

    ld.add_action(OpaqueFunction(function=_make_nodes))

    return ld
