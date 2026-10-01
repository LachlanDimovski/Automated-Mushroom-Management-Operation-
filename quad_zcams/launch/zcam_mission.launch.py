"""cage_mission.launch.py with the z-camera Parrot: same sim, rover and arguments, but the quad
carries a down and an up depth camera and runs zcam_quad_node.

    ros2 launch quad_zcams zcam_mission.launch.py world:=forest_flat headless:=true rviz:=false meta_goals:=false
"""
import importlib.util
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('41068_ignition_bringup')
HERE = get_package_share_directory('quad_zcams')


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _bringup_helpers():
    return _load(os.path.join(BRINGUP, 'launch', '41068_ignition.launch.py'), 'ignition_bringup_launch')


def _cage_mission():
    """cage_mission.launch.py with its capture node swapped for rover_capture_node (same parameters)."""
    mod = _load(os.path.join(get_package_share_directory('ammo_mission_control'), 'launch', 'cage_mission.launch.py'),
                'cage_mission_launch')
    stock = mod._nodes

    def _nodes(context):
        out = [n for n in stock(context) if getattr(n, 'node_executable', None) != 'cage_capture_node']
        world = LaunchConfiguration('world').perform(context)
        trunks, goals_x, goals_y = [0.0, 3.0, 5.0, 0.0], [0.0], [0.0]
        meta = os.path.join(BRINGUP, 'worlds', f'{world}.meta.json')
        if os.path.exists(meta):
            d = json.load(open(meta))
            trunks = [v for o in d['objects'] if o['type'] == 'tree' for v in (o['x'], o['y'])]
            if LaunchConfiguration('meta_goals').perform(context).lower() == 'true':
                goals_x = [m['x'] for m in d['mushrooms']] or [0.0]
                goals_y = [m['y'] for m in d['mushrooms']] or [0.0]
        return out + [Node(package='quad_zcams', executable='rover_capture_node', output='screen',
                           parameters=[{'use_sim_time': True, 'world': os.path.basename(world), 'trunks': trunks,
                                        'goals_x': goals_x, 'goals_y': goals_y,
                                        'count': int(LaunchConfiguration('count').perform(context)),
                                        'tool': LaunchConfiguration('tool').perform(context)}])]
    mod._nodes = _nodes
    return mod.generate_launch_description()


def _quad(context):
    # same parameters cage_mission.launch.py gives quad_search_node
    world = LaunchConfiguration('world').perform(context)
    trunks, half, ground_z = [0.0, 3.0, 5.0, 0.0], 6.5, 0.0
    meta = os.path.join(BRINGUP, 'worlds', f'{world}.meta.json')
    if os.path.exists(meta):
        d = json.load(open(meta))
        trunks = [v for o in d['objects'] if o['type'] == 'tree' for v in (o['x'], o['y'])]
        half = d['world']['playable_size_m'] / 2.0 - 4.5
        ground_z = d['spawn_clearing']['ground_z']
    qwp = [float(v) for v in LaunchConfiguration('quad_waypoints').perform(context).split(',') if v.strip()]
    sdf = os.path.join(BRINGUP, 'worlds', f'{world}.sdf')  # an absolute world path replaces the prefix
    planner = LaunchConfiguration('planner').perform(context).lower() == 'true'
    return [Node(package='quad_zcams', executable='zcam_quad_node', output='screen',
                 parameters=[{'use_sim_time': True, 'trunks': trunks, 'waypoints': qwp or [0.0], 'ground_z': ground_z,
                              'search_area': [-half, half, -half, half, 3.0],
                              'world_sdf': sdf if planner and os.path.exists(sdf) else '',
                              'cruise_alt': float(LaunchConfiguration('cruise_alt').perform(context)),
                              'clearance': float(LaunchConfiguration('clearance').perform(context))}])]


def generate_launch_description():
    ld = LaunchDescription()
    for name, default in (('world', 'simple_trees'), ('rviz', 'true'), ('headless', 'false'), ('count', '0'),
                          ('meta_goals', 'true'), ('yolo', 'true'), ('tool', 'gripper'), ('quad_waypoints', ''),
                          ('mushrooms', ''), ('husky_yolo', 'false'), ('hazard', 'false'), ('use_sim_time', 'True'),
                          ('planner', 'true'),  # planner:=false = the first z-camera trial (straight legs)
                          ('cruise_alt', '2.0'), ('clearance', '1.0')):  # leaf-mesh dense forest: 1.0 / 0.6
        ld.add_action(DeclareLaunchArgument(name, default_value=default))
    # everything but the stock quad (its URDF and search node are swapped for the z-camera ones below)
    ld.add_action(IncludeLaunchDescription(LaunchDescriptionSource(_cage_mission()),
                                           launch_arguments={'parrot': 'false'}.items()))
    ld.add_action(SetLaunchConfiguration('zcam_parrot', 'true'))
    base = _bringup_helpers()
    base.add_robot(
        ld, pkg_path=HERE, config_path=os.path.join(BRINGUP, 'config'), use_sim_time=LaunchConfiguration('use_sim_time'),
        enabled_arg='zcam_parrot', namespace='parrot1', frame_prefix='parrot1_', gz_model_name='parrot1',
        xacro_parts=['urdf', 'parrot_zcams.urdf.xacro'], bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml', x='2.0', y='0.0', z='0.8', spawn_delay=6.0)
    ld.add_action(Node(package='ros_ign_bridge', executable='parameter_bridge', namespace='parrot1',
                       name='zcam_bridge', output='screen',
                       parameters=[{'config_file': os.path.join(HERE, 'config', 'gazebo_bridge_parrot1_zcams.yaml'),
                                    'use_sim_time': LaunchConfiguration('use_sim_time')}]))
    base.add_perception(ld, use_sim_time=LaunchConfiguration('use_sim_time'), robot_arg='zcam_parrot',
                        robot_namespace='parrot1', start_delay=11.0, core='7')
    ld.add_action(OpaqueFunction(function=_quad))
    return ld
