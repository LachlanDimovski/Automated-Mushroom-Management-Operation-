"""Skidsteer waypoint mission: sim + Nav2 (+ RViz), ground-truth localisation,
YOLO/depth target selector and the cage capture node.

    ros2 launch ammo_mission_control cage_mission.launch.py                   # simple_trees, RViz
    ros2 launch ammo_mission_control cage_mission.launch.py world:=forest_random
    ros2 launch ammo_mission_control cage_mission.launch.py count:=4          # spawn 4 random test mushrooms
    ros2 launch ammo_mission_control cage_mission.launch.py headless:=true rviz:=false

Give it waypoints with RViz Publish Point / 2D Goal Pose, or on
husky1/cage/waypoints. In forest_random the world's own mushrooms (from
forest_random.meta.json) are queued as the initial waypoints and its trunks
are passed for standoff planning.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP = get_package_share_directory('41068_ignition_bringup')


def _nodes(context):
    world = LaunchConfiguration('world').perform(context)
    trunks, goals_x, goals_y = [0.0, 3.0, 5.0, 0.0], [0.0], [0.0]  # simple_trees defaults
    meta = os.path.join(BRINGUP, 'worlds', f'{world}.meta.json')
    if os.path.exists(meta):
        d = json.load(open(meta))
        trunks = [v for o in d['objects'] if o['type'] == 'tree' for v in (o['x'], o['y'])]
        goals_x = [m['x'] for m in d['mushrooms']] or [0.0]
        goals_y = [m['y'] for m in d['mushrooms']] or [0.0]
    sim = {'use_sim_time': True}
    return [
        Node(package='ammo_mission_control', executable='truth_localizer', output='screen', parameters=[sim],
             remappings=[('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')]),
        Node(package='skidsteer_perception', executable='target_selector', output='screen',
             parameters=[{**sim, 'drive_to_target': True}]),
        Node(package='ammo_mission_control', executable='cage_capture_node', output='screen',
             parameters=[{**sim, 'world': world, 'trunks': trunks, 'goals_x': goals_x, 'goals_y': goals_y,
                          'count': ParameterValue(LaunchConfiguration('count'), value_type=int)}]),
    ]


def generate_launch_description():
    ld = LaunchDescription()
    for name, default in (('world', 'simple_trees'), ('rviz', 'true'), ('headless', 'false'), ('count', '0')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(BRINGUP, 'launch', 'forest_world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'nav2': 'true',
                          'ground_truth_localization': 'true', 'rviz': LaunchConfiguration('rviz'),
                          'headless': LaunchConfiguration('headless')}.items()))
    ld.add_action(OpaqueFunction(function=_nodes))
    return ld
