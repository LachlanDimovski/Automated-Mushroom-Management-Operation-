"""Standalone test launch for the skid-steer Husky.

Spawns only the husky1 platform (no parrot), starts the YOLO detector so
detections can be watched live on husky1/detections/image and
husky1/target_info, and drives the robot either autonomously
(skidsteer_controller_node - waypoint following, with visual servo to
chase/stop at a detected target) or, if teleop:=True, via an xterm running
teleop_twist_keyboard so it can be driven freely by hand instead.
"""
import math
import os
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Object types scattered into the world: (model name, how many, is a YOLO
# driveable target or just clutter the robot should ignore, spawn height).
# Spawn height depends on where the model's origin sits: the decoy primitives
# are centred on their own origin so they need lifting clear of the ground,
# while the mushroom mesh has its base baked to z=0 and spawns flush at 0.
SCATTER_MODELS = [
    ('mushroom_amanita', 4, True, 0.0),
    ('decoy_box', 3, False, 0.3),
    ('decoy_cylinder', 3, False, 0.3),
]
# Keep objects out of the husky1 spawn point and spread them over the world.
SCATTER_RADIUS_MIN_M = 3.0
SCATTER_RADIUS_MAX_M = 9.0
SCATTER_SPAWN_DELAY_S = 8.0


def scatter_object_nodes(use_sim_time):
    """Build spawn actions that drop the target/decoy models at random poses.

    Poses are randomised once per `ros2 launch` invocation (at launch
    description build time), so every run gets a different scattered layout.
    """
    nodes = []
    for model_name, count, _is_target, spawn_z in SCATTER_MODELS:
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
                name=f'spawn_{model_name}_{i}',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-file', model_file,
                    '-name', f'{model_name}_{i}',
                    '-x', str(x), '-y', str(y), '-z', str(spawn_z),
                ],
            ))
    return nodes


def generate_launch_description():
    # VS Code's snap sandbox injects GTK/locale env vars that shadow system
    # libs with an incompatible libpthread, crashing any Qt/GTK app (rviz2,
    # rqt) launched from an integrated terminal. Strip them for children.
    for var in (
        'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
        'GIO_MODULE_DIR', 'LOCPATH', 'GSETTINGS_SCHEMA_DIR',
    ):
        os.environ.pop(var, None)

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    yolo = LaunchConfiguration('yolo')
    teleop = LaunchConfiguration('teleop')
    detection_view = LaunchConfiguration('detection_view')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo'],
    ))
    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Run Gazebo server-only (no GUI window); use RViz for visualization instead',
    ))
    ld.add_action(DeclareLaunchArgument(
        'yolo',
        default_value='True',
        description='Flag to launch the YOLO detector on top of the driving husky',
    ))
    ld.add_action(DeclareLaunchArgument(
        'teleop',
        default_value='False',
        description='Drive husky1 by hand via keyboard teleop instead of the autonomous controller',
    ))
    ld.add_action(DeclareLaunchArgument(
        'detection_view',
        default_value='True',
        description='Pop up a standalone rqt_image_view window on husky1/detections/image',
    ))

    # Bring up Gazebo + husky1 only, with RViz for viewing the camera/scan/detections.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('41068_ignition_bringup'),
            'launch',
            '41068_ignition.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'True',
            'slam': 'False',
            'nav2': 'False',
            'world': world,
            'headless': headless,
            'husky': 'True',
            'parrot': 'False',
            # Starts perception/yolo_node -> husky1/detections.
            'yolo': yolo,
        }.items(),
    ))

    # Keyboard teleop needs a real terminal for stdin, so run it in its own xterm
    # rather than under `ros2 launch` directly. Only started when teleop:=True.
    ld.add_action(ExecuteProcess(
        cmd=[
            'xterm', '-hold', '-e',
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            # Publishes to the "raw" topic - collision_guard_node is the
            # final gate to husky1/cmd_vel, so a human driver gets the same
            # obstacle protection the autonomous controller has.
            '--ros-args', '-r', 'cmd_vel:=husky1/cmd_vel_raw',
            '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen',
        condition=IfCondition(teleop),
    ))

    # Default: autonomous waypoint following, switching to visual-servo
    # chase/stop whenever target_selector reports a fresh target.
    ld.add_action(Node(
        package='skidsteer_controller',
        executable='skidsteer_controller_node',
        name='skidsteer_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(teleop),
    ))

    # Always-on safety layer: gates husky1/cmd_vel_raw -> husky1/cmd_vel,
    # blocking forward motion into a close obstacle regardless of whether
    # teleop or the autonomous controller is driving.
    ld.add_action(Node(
        package='skidsteer_controller',
        executable='collision_guard_node',
        name='collision_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # yolo_node is started by the 41068_ignition.launch.py include above.
    # Husky-only: range husky1/detections against the depth camera and publish
    # the best target on husky1/target_info for skidsteer_controller.
    ld.add_action(Node(
        package='skidsteer_perception',
        executable='target_selector',
        name='target_selector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(yolo),
    ))

    # Standalone window showing the annotated YOLO detections live, separate
    # from rviz so it's not competing for space with the 3D view.
    ld.add_action(Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='detections_view',
        output='screen',
        arguments=['/husky1/detections/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(detection_view),
    ))

    # Scatter a random mix of driveable targets (mushrooms) and decoy
    # clutter (boxes/cylinders) around the world once Gazebo + husky1 are up.
    ld.add_action(TimerAction(
        period=SCATTER_SPAWN_DELAY_S,
        actions=scatter_object_nodes(use_sim_time),
    ))

    return ld
