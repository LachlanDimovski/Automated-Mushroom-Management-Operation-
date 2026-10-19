"""zcam_mission.launch.py with a fleet: `quads` z-camera Parrots (parrot1, parrot2, ...) each searching its own sector,
`rovers` Huskies, and fleet_coordinator, which gives every mushroom to exactly one rover and ends the mission.

    ros2 launch quad_zcams fleet_mission.launch.py world:=<abs path>/river_moderate_aus headless:=true rviz:=false \
        meta_goals:=false quads:=2 rovers:=2

Topics: the quads report to fleet/reports and hear fleet/done and fleet/missed; the coordinator sends each mushroom to
<rover>/cage/waypoints; the rovers route on fleet/ground_obstacles (every quad's map).

YOLO: one fleet_yolo_pool process (OpenVINO, `pool_workers` pinned cores, default 1 per 3 quads) serves every quad's
camera, and husky1's while husky_yolo is true; yolo_pool:=false brings back a yolo_node per robot. The pool cut YOLO
RAM from 6.2 to 1.1 GB with 6 quads and passed every fleet benchmark run (Documents/41068_evidence/fleet_bench).

Rover k > 1 runs the same nodes and configs as husky1 under the namespace rk (topics /rk/husky1/..., its own TF tree
on /rk/husky1/tf, Gazebo model huskyk), so nothing written for husky1 needs a robot parameter.
"""
import importlib.util
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP = get_package_share_directory('41068_ignition_bringup')
HERE = get_package_share_directory('quad_zcams')
MAX_QUADS = 4
OPENVINO = os.path.join(HERE, 'models', 'amanita_river_v4_yolo11n_openvino_model')
MAX_ROVERS = 5
SPAWNS = [(2.0, 0.0), (-2.0, 0.0), (0.0, 2.0), (0.0, -2.0)]  # inside the 4 m spawn clearing, clear of the rover
ROVER_SPAWNS = [(0.0, 0.0), (2.5, -2.5), (-2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)]  # off the quads' axes
QUAD_TOPICS = ['cmd_vel', 'odometry', 'scan', 'detections', 'camera/depth/image', 'down_depth/image', 'up_depth/image',
               'teleop_active', 'teleop_mark', 'landed', 'agl', 'ground_obstacles']
FLEET_TOPICS = [('husky1/cage/waypoints', 'fleet/reports'), ('husky1/cage/done', 'fleet/done'),
                ('husky1/cage/missed', 'fleet/missed')]


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _configs(ns):
    """parrot1's bridge and localization files rewritten for ns, in a temp dir."""
    out = tempfile.mkdtemp(prefix=f'fleet_{ns}_')
    for src in (os.path.join(BRINGUP, 'config', 'gazebo_bridge_parrot1.yaml'),
                os.path.join(BRINGUP, 'config', 'robot_localization_parrot1.yaml'),
                os.path.join(HERE, 'config', 'gazebo_bridge_parrot1_zcams.yaml')):
        open(os.path.join(out, os.path.basename(src).replace('parrot1', ns)), 'w').write(
            open(src).read().replace('parrot1', ns))
    return out


def _rover_configs(k):
    """husky1's bridge file pointed at Gazebo model huskyk, in a temp dir."""
    out = tempfile.mkdtemp(prefix=f'fleet_husky{k}_')
    open(os.path.join(out, 'gazebo_bridge.yaml'), 'w').write(
        open(os.path.join(BRINGUP, 'config', 'gazebo_bridge_husky1.yaml')).read().replace('/model/husky1/', f'/model/husky{k}/'))
    return out


def _count(context, name, top):
    return max(1, min(top, int(LaunchConfiguration(name).perform(context))))


def _rover_names(n):
    return ['husky1'] + [f'r{k}/husky1' for k in range(2, n + 1)]


def _pool(context):
    if LaunchConfiguration('yolo_pool').perform(context).lower() != 'true':
        return []
    n = _count(context, 'quads', MAX_QUADS)
    w = int(LaunchConfiguration('pool_workers').perform(context) or 0) or (n + 2) // 3
    model = LaunchConfiguration('model_path').perform(context) or os.path.join(
        get_package_share_directory('perception'), 'models', 'amanita_muscaria_yolo11n.pt')
    robots = [f'parrot{k + 1}' for k in range(n)]
    if LaunchConfiguration('husky_yolo').perform(context).lower() == 'true':
        robots.append('husky1')
    return [Node(package='quad_zcams', executable='fleet_yolo_pool', output='screen',
                 parameters=[{'use_sim_time': True, 'robots': robots, 'model_path': model,
                              'openvino_path': OPENVINO if 'amanita_river_v4' in model else '',  # river model's export
                              'imgsz': int(LaunchConfiguration('imgsz').perform(context)), 'workers': w}],
                 additional_env={'YOLO_CORES': ','.join(str(7 - i) for i in range(w))})]


def generate_launch_description():
    zm = _load(os.path.join(HERE, 'launch', 'zcam_mission.launch.py'), 'zcam_mission_launch')
    real_node = zm.Node
    rover_kw = {}

    def node(**kw):  # the rover routes on every quad's map; the coordinator decides when the mission is complete
        if kw.get('executable') == 'rover_capture_node':
            kw['parameters'] = [dict(kw['parameters'][0], total=10 ** 6, end_sim=False)]
            kw['remappings'] = [('parrot1/ground_obstacles', 'fleet/ground_obstacles'), ('parrot1/landed', 'fleet/landed')]
            rover_kw.update(kw)
        return real_node(**kw)

    def quads(context):
        n = max(1, min(MAX_QUADS, int(LaunchConfiguration('quads').perform(context))))
        captured = []
        zm.Node = lambda **kw: captured.append(kw)
        try:
            zm._quad_stock(context)
        finally:
            zm.Node = node
        kw = captured[0]
        out = []
        standdown = LaunchConfiguration('standdown').perform(context) == 'true'
        for k in range(n):
            ns = f'parrot{k + 1}'
            params = dict(kw['parameters'][0], fleet_name=ns, sector=k, sectors=n)
            remaps = FLEET_TOPICS + [(f'parrot1/{t}', f'{ns}/{t}') for t in QUAD_TOPICS if ns != 'parrot1']
            # home and land once the search is done, not at mission complete (quad_standdown); parrot1 keeps
            # searching: a YOLO false positive on the list hid a real mushroom from the stood-down quads (q4r1_ub)
            if standdown and k > 0:
                remaps.append(('ammo/mission_complete', 'fleet/search_done'))
            out.append(Node(package='quad_zcams', executable='fleet_quad_node', name=f'fleet_quad_{ns}',
                            output='screen', parameters=[params], remappings=remaps))
        if standdown:
            out.append(Node(package='quad_zcams', executable='quad_standdown', output='screen',
                            parameters=[{'use_sim_time': True, 'world': LaunchConfiguration('world').perform(context)}]))
        out.append(Node(package='quad_zcams', executable='fleet_coordinator', output='screen',
                        parameters=[{'use_sim_time': True, 'rovers': _rover_names(_count(context, 'rovers', MAX_ROVERS)),
                                     'quads': [f'parrot{k + 1}' for k in range(n)], 'total': _total(context)}]))
        return out

    def extra_quads(context):
        """parrot2.. spawned, bridged and given YOLO like zcam_mission does parrot1."""
        n = max(1, min(MAX_QUADS, int(LaunchConfiguration('quads').perform(context))))
        ld = LaunchDescription()
        base = zm._bringup_helpers()
        for k in range(1, n):
            ns, (x, y) = f'parrot{k + 1}', SPAWNS[k]
            cfg = _configs(ns)
            base.add_robot(
                ld, pkg_path=HERE, config_path=cfg, use_sim_time=LaunchConfiguration('use_sim_time'),
                enabled_arg='zcam_parrot', namespace=ns, frame_prefix=f'{ns}_', gz_model_name=ns,
                xacro_parts=['urdf', 'parrot_zcams.urdf.xacro'], bridge_config=f'gazebo_bridge_{ns}.yaml',
                localization_config=f'robot_localization_{ns}.yaml', x=str(x), y=str(y), z='0.8',
                spawn_delay=6.0 + 2.0 * k)
            ld.add_action(Node(package='ros_ign_bridge', executable='parameter_bridge', namespace=ns,
                               name='zcam_bridge', output='screen',
                               parameters=[{'config_file': os.path.join(cfg, f'gazebo_bridge_{ns}_zcams.yaml'),
                                            'use_sim_time': LaunchConfiguration('use_sim_time')}]))
            base.add_perception(ld, use_sim_time=LaunchConfiguration('use_sim_time'), robot_arg='zcam_parrot',
                                robot_namespace=ns, start_delay=11.0 + k, core=str(6 - k))  # husky 6, parrot1 7
        return list(ld.entities)

    def _total(context):
        world = LaunchConfiguration('world').perform(context)
        meta = os.path.join(BRINGUP, 'worlds', f'{world}.meta.json')
        return len(json.load(open(meta)).get('mushrooms', [])) if os.path.exists(meta) else 0

    def extra_rovers(context):
        """husky2.. with everything husky1 has: description, EKF, bridge, Nav2, ground-truth localizer, collision guard,
        capture node (and YOLO with rover_yolo:=true)."""
        if not rover_kw:
            raise RuntimeError('fleet_mission: husky1\'s rover_capture_node was not created before the extra rovers')
        sim = LaunchConfiguration('use_sim_time')
        out = []
        for k in range(2, _count(context, 'rovers', MAX_ROVERS) + 1):
            top, ns, (x, y) = f'r{k}', f'r{k}/husky1', ROVER_SPAWNS[k - 1]
            tf = [('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]
            desc = ParameterValue(Command(['xacro ', os.path.join(BRINGUP, 'urdf_husky', 'husky.urdf.xacro'),
                                           ' prefix:=husky1_ gz_model_name:=', f'husky{k}']), value_type=str)
            nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(BRINGUP, 'launch', '41068_navigation.launch.py')),
                launch_arguments={'use_sim_time': sim, 'namespace': ns, 'config_filename_suffix': '_husky1',
                                  'slam': LaunchConfiguration('slam'), 'nav2': LaunchConfiguration('nav2'),
                                  'ground_truth_localization': LaunchConfiguration('ground_truth_localization')}.items())
            ld = LaunchDescription()
            if LaunchConfiguration('rover_yolo').perform(context).lower() == 'true':
                zm._bringup_helpers().add_perception(ld, use_sim_time=sim, robot_arg='husky', robot_namespace=ns,
                                                     start_delay=8.0 + k)
            out.append(GroupAction(scoped=True, actions=[
                SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_nav_guarded'),
                Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
                     parameters=[{'robot_description': desc, 'use_sim_time': sim}],
                     remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
                Node(package='robot_localization', executable='ekf_node', namespace=ns, name='robot_localization',
                     parameters=[os.path.join(BRINGUP, 'config', 'robot_localization_husky1.yaml'), {'use_sim_time': sim}],
                     remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('odometry/filtered', 'odom')]),
                Node(package='ros_ign_bridge', executable='parameter_bridge', namespace=ns, name='gazebo_bridge',
                     parameters=[{'config_file': os.path.join(_rover_configs(k), 'gazebo_bridge.yaml'),
                                  'use_sim_time': sim}]),
                TimerAction(period=3.0 + 2.0 * k, actions=[Node(
                    package='ros_ign_gazebo', executable='create', name=f'spawn_husky{k}', output='screen',
                    arguments=['-topic', f'/{ns}/robot_description', '-name', f'husky{k}',
                               '-x', str(x), '-y', str(y), '-z', '0.4'])]),
                TimerAction(period=7.0, actions=[nav]),
                Node(package='ammo_mission_control', executable='truth_localizer', namespace=top, output='screen',
                     parameters=[{'use_sim_time': True}], remappings=tf),
                Node(package='skidsteer_controller', executable='collision_guard_node', namespace=top, output='screen',
                     parameters=[{'use_sim_time': True}], remappings=tf),
                Node(package='quad_zcams', executable='rover_capture_node', namespace=top, output='screen',
                     parameters=rover_kw['parameters'],
                     remappings=[('parrot1/ground_obstacles', '/fleet/ground_obstacles'),
                                 ('parrot1/landed', '/fleet/landed'),
                                 ('ammo/mission_complete', '/ammo/mission_complete')]),
                *ld.entities]))
        return out

    zm.Node = node
    zm._quad_stock, zm._quad = zm._quad, quads
    return LaunchDescription([DeclareLaunchArgument('quads', default_value='2'),
                              DeclareLaunchArgument('standdown', default_value='true'),
                              DeclareLaunchArgument('rovers', default_value='1'),
                              DeclareLaunchArgument('rover_yolo', default_value='false'),  # YOLO on rovers 2..
                              DeclareLaunchArgument('yolo_pool', default_value='true'),
                              DeclareLaunchArgument('pool_workers', default_value='0'),
                              DeclareLaunchArgument('model_path', default_value=''),
                              # with the pool no yolo_node runs (husky1's too goes to the pool)
                              OpaqueFunction(function=lambda c: [SetLaunchConfiguration('yolo', 'false')]
                                             if LaunchConfiguration('yolo_pool').perform(c).lower() == 'true' else []),
                              *zm.generate_launch_description().entities, OpaqueFunction(function=extra_quads),
                              OpaqueFunction(function=extra_rovers), OpaqueFunction(function=_pool)])
