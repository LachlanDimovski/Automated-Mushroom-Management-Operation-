"""Bring up the randomised forest world (worlds/forest_random.sdf).

Standalone counterpart to 41068_ignition.launch.py, pointed at the generated
forest world instead of simple_trees / large_demo. It does not modify or depend
on the behaviour of the existing launch files - it just reuses their robot
bringup helpers so husky1 / parrot1 come up identically.

Generate the world first (any seed):

    python3 src/RS1_Team5_/41068_ignition_bringup/scripts/generate_forest_world.py --seed 7
    colcon build --packages-select 41068_ignition_bringup

Then:

    ros2 launch 41068_ignition_bringup forest_world.launch.py
    ros2 launch 41068_ignition_bringup forest_world.launch.py nav2:=true rviz:=true
    ros2 launch 41068_ignition_bringup forest_world.launch.py parrot:=true

Arguments mirror 41068_ignition.launch.py (use_sim_time, husky, parrot, slam,
nav2, rviz, yolo, headless). `world` defaults to forest_random but accepts any
worlds/<name>.sdf you have generated with --name.
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _load_base_launch():
    """Import 41068_ignition.launch.py as a module to reuse its helpers.

    The filename starts with a digit and has a double extension, so it cannot be
    imported normally - load it by path. Only module-level function definitions
    run at import time (no side effects), so this is safe.
    """
    path = os.path.join(
        get_package_share_directory('41068_ignition_bringup'),
        'launch', '41068_ignition.launch.py')
    spec = importlib.util.spec_from_file_location('_forest_base_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_base = _load_base_launch()


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    args = [
        ('use_sim_time', 'True', 'Flag to enable use_sim_time'),
        ('rviz', 'False', 'Flag to launch RViz'),
        ('slam', 'False', 'Flag to launch SLAM Toolbox for each enabled robot'),
        ('nav2', 'False', 'Flag to launch Nav2 for each enabled robot. Nav2 also starts SLAM.'),
        ('ground_truth_localization', 'False', 'Skip SLAM; map->odom from an external ground-truth localizer'),
        ('husky', 'True', 'Launch the Husky UGV instance in namespace /husky1'),
        ('parrot', 'False', 'Launch the Parrot drone instance in namespace /parrot1'),
        ('yolo', 'True', 'Run the shared YOLO detector for each enabled robot'),
        ('headless', 'False', 'Run Gazebo server-only (no GUI window)'),
        ('world', 'forest_random', 'Generated world name under worlds/ (no .sdf)'),
    ]
    for name, default, desc in args:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=desc))

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Same shared server systems (Physics, Sensors, SceneBroadcaster, Contact)
    # the main launch uses - the forest world carries no plugin blocks.
    server_config_file = PathJoinSubstitution([config_path, 'ignition_server.config'])
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_SERVER_CONFIG_PATH', server_config_file))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_config_file))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py',
        ])),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([
                    pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf'],
                ]),
                PythonExpression([
                    "' -s -r' if '", LaunchConfiguration('headless'),
                    "'.lower() in ", "['true', '1', 'yes', 'on']", " else ' -r'",
                ]),
            ]
        }.items(),
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    # Robots, navigation, perception and RViz - identical wiring to
    # 41068_ignition.launch.py (same spawn poses and start delays).
    _base.add_robot(
        ld, pkg_path=pkg_path, config_path=config_path, use_sim_time=use_sim_time,
        enabled_arg='husky', namespace='husky1', frame_prefix='husky1_',
        gz_model_name='husky1', xacro_parts=['urdf_husky', 'husky.urdf.xacro'],
        bridge_config='gazebo_bridge_husky1.yaml',
        localization_config='robot_localization_husky1.yaml',
        x='0.0', y='0.0', z='0.4', spawn_delay=3.0,
    )
    _base.add_robot(
        ld, pkg_path=pkg_path, config_path=config_path, use_sim_time=use_sim_time,
        enabled_arg='parrot', namespace='parrot1', frame_prefix='parrot1_',
        gz_model_name='parrot1', xacro_parts=['urdf_parrot', 'parrot.urdf.xacro'],
        bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml',
        x='2.0', y='0.0', z='0.8', spawn_delay=6.0,
    )

    _base.add_navigation_instance(
        ld, pkg_path=pkg_path, use_sim_time=use_sim_time,
        robot_arg='husky', robot_namespace='husky1', start_delay=7.0)
    # ponytail: the quad is flown by direct velocity control (quad_search_node /
    # ps4_drone_test.py), never NavigateToPose - a Nav2 stack for it is dead
    # weight with no ground-truth localizer feeding it a map frame, and was
    # spamming "parrot1_map does not exist" plus adding enough load to disrupt
    # husky1's own TF timing. Give it none regardless of the shared nav2 flag.

    _base.add_perception(
        ld, use_sim_time=use_sim_time,
        robot_arg='husky', robot_namespace='husky1', start_delay=8.0)
    _base.add_perception(
        ld, use_sim_time=use_sim_time,
        robot_arg='parrot', robot_namespace='parrot1', start_delay=11.0)

    _base.add_rviz_instance(
        ld, config_path=config_path, use_sim_time=use_sim_time,
        robot_arg='husky', robot_namespace='husky1',
        rviz_config='41068_husky1.rviz', start_delay=11.0)
    _base.add_rviz_instance(
        ld, config_path=config_path, use_sim_time=use_sim_time,
        robot_arg='parrot', robot_namespace='parrot1',
        rviz_config='41068_parrot1.rviz', start_delay=13.0)

    return ld
