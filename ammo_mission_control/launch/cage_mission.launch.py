"""Skidsteer waypoint mission: sim + Nav2 (+ RViz), ground-truth localisation,
YOLO/depth target selector and the cage capture node.

    ros2 launch ammo_mission_control cage_mission.launch.py                   # simple_trees, RViz
    ros2 launch ammo_mission_control cage_mission.launch.py world:=forest_random
    ros2 launch ammo_mission_control cage_mission.launch.py count:=4          # spawn 4 random test mushrooms
    ros2 launch ammo_mission_control cage_mission.launch.py headless:=true rviz:=false
    ros2 launch ammo_mission_control cage_mission.launch.py parrot:=false count:=6  # husky-only, isolate capture debugging
    ros2 launch ammo_mission_control cage_mission.launch.py quad_waypoints:=4,-3,-3,-3.5   # quad flies these legs instead of the lawnmower

With parrot:=true the quad flies its legs, holds on a YOLO hit, publishes the
projected ground point once still, the rover collects it, the quad resumes.
quad:=zcam (default) runs quad_zcams' zcam_mission.launch.py: the z-camera quad
plans round everything at cruise height and never climbs over canopies.
quad:=stock runs quad_search_node (trunk detours, climb-over recovery).

Give it waypoints with RViz Publish Point / 2D Goal Pose, or on
husky1/cage/waypoints. In forest_random the world's own mushrooms (from
forest_random.meta.json) are queued as the initial waypoints and its trunks
are passed for standoff planning.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP = get_package_share_directory('41068_ignition_bringup')
# The operator GUI freezes the mission nodes with SIGSTOP (Pause, operator goals). A node
# frozen past Fast DDS's 20 s lease is dropped by the other processes and not always re-matched (it loses /clock and
# Nav2). The GUI sets AMMO_FREEZABLE_PROFILE for them: a 1 h lease, loopback UDP only. ROS_LOCALHOST_ONLY=1 would put
# shared memory back, so it is off here (the profile is loopback only).
FREEZABLE_ENV = ({'FASTRTPS_DEFAULT_PROFILES_FILE': os.environ['AMMO_FREEZABLE_PROFILE'], 'ROS_LOCALHOST_ONLY': '0'}
                 if os.environ.get('AMMO_FREEZABLE_PROFILE') else {})


def _nodes(context):
    world = LaunchConfiguration('world').perform(context)
    trunks, goals_x, goals_y = [0.0, 3.0, 5.0, 0.0], [0.0], [0.0]  # simple_trees defaults
    meta = os.path.join(BRINGUP, 'worlds', f'{world}.meta.json')
    if os.path.exists(meta):
        d = json.load(open(meta))
        trunks = [v for o in d['objects'] if o['type'] == 'tree' for v in (o['x'], o['y'])]
        if LaunchConfiguration('meta_goals').perform(context).lower() == 'true':  # false = quad dispatches only
            goals_x = [m['x'] for m in d['mushrooms']] or [0.0]
            goals_y = [m['y'] for m in d['mushrooms']] or [0.0]
    # 4.5 m in from the boundary tree ring (at 3 m the quad stalled in the corners); simple_trees grass is 15 x 15 m
    half = d['world']['playable_size_m'] / 2.0 - 4.5 if os.path.exists(meta) else 6.5
    ground_z = d['spawn_clearing']['ground_z'] if os.path.exists(meta) else 0.0
    qwp = [float(v) for v in LaunchConfiguration('quad_waypoints').perform(context).split(',') if v.strip()]
    # demo mushrooms 'x,y,x,y,...' spawned once Gazebo is up - not queued, the quad has to find them
    mp = [float(v) for v in LaunchConfiguration('mushrooms').perform(context).split(',') if v.strip()]
    sdf = os.path.join(BRINGUP, 'models', 'mushroom_amanita', 'model.sdf')
    spawns = [Node(package='ros_ign_gazebo', executable='create', output='log',
                   arguments=['-file', sdf, '-name', f'demo_mushroom_{i}', '-x', str(x), '-y', str(y), '-z', '0.0'])
              for i, (x, y) in enumerate(zip(mp[0::2], mp[1::2]), 1)]
    sim = {'use_sim_time': True}
    parrot_on = LaunchConfiguration('parrot').perform(context).lower() == 'true'
    quad = [] if not parrot_on else [
        Node(package='ammo_mission_control', executable='quad_search_node', output='screen', additional_env=FREEZABLE_ENV,
             parameters=[{**sim, 'trunks': trunks, 'waypoints': qwp or [0.0], 'ground_z': ground_z,
                          'search_area': [-half, half, -half, half, 3.0]}]),  # rows 3 m apart: YOLO range is ~3.5 m
    ]
    return ([TimerAction(period=12.0, actions=spawns)] if spawns else []) + quad + [
        Node(package='ammo_mission_control', executable='truth_localizer', output='screen', parameters=[sim],
             remappings=[('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')]),
        # cage_capture_node now publishes cmd_vel_raw (not cmd_vel directly) so
        # this guard can still stop it for low obstacles (fallen trees/logs)
        # that forest_random's terrain puts below Nav2's costmap.
        Node(package='skidsteer_controller', executable='collision_guard_node', output='screen', parameters=[sim],
             additional_env=FREEZABLE_ENV,
             remappings=[('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')]),  # camera pose for the depth check
        Node(package='ammo_mission_control', executable='cage_capture_node', output='screen', additional_env=FREEZABLE_ENV,
             parameters=[{**sim, 'world': os.path.basename(world), 'trunks': trunks, 'goals_x': goals_x, 'goals_y': goals_y,
                          'count': ParameterValue(LaunchConfiguration('count'), value_type=int),
                          'tool': LaunchConfiguration('tool')}]),
    ]


def generate_launch_description():
    ld = LaunchDescription()
    for name, default in (('world', 'simple_trees'), ('rviz', 'true'), ('headless', 'false'),
                          ('count', '0'), ('parrot', 'true'), ('meta_goals', 'true'), ('yolo', 'true'),
                          ('tool', 'gripper'),
                          # quad flight plan as 'x0,y0,x1,y1,...'; empty = lawnmower over the world
                          ('quad_waypoints', ''), ('mushrooms', ''), ('husky_yolo', 'true'),
                          ('hazard', 'false'), ('quad', 'zcam')):  # hazard: Henry's COCO person detector + hazard_brake on the rover
        ld.add_action(DeclareLaunchArgument(name, default_value=default))
    ld.add_action(OpaqueFunction(function=_mission))
    return ld


def _mission(context):
    if (LaunchConfiguration('quad').perform(context) == 'zcam'
            and LaunchConfiguration('parrot').perform(context).lower() == 'true'):
        # zcam_mission includes this file again with parrot:=false and adds its own quad
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('quad_zcams'), 'launch', 'zcam_mission.launch.py')))]
    # Nav2's smoothed output goes through collision_guard's depth check instead of straight to the wheels
    return [SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_guarded'), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(BRINGUP, 'launch', '41068_ignition.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'nav2': 'true',
                          'parrot': LaunchConfiguration('parrot'), 'yolo': LaunchConfiguration('yolo'),
                          'husky_yolo': LaunchConfiguration('husky_yolo'),
                          'hazard': LaunchConfiguration('hazard'),
                          'ground_truth_localization': 'true', 'rviz': LaunchConfiguration('rviz'),
                          'headless': LaunchConfiguration('headless')}.items()),
            OpaqueFunction(function=lambda c: _nodes(c))]  # looked up at run time: zcam_mission swaps _nodes
