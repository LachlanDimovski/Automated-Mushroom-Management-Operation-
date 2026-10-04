"""One-command demo of the AMMO perception and navigation pipeline.

Brings up everything needed to film evidence of the system working:

    both detectors       husky1/combined/image     (one view, both drawn)
    mushrooms only       husky1/detections/image   (fine-tuned YOLO)
    people only          husky1/hazards/image      (stock COCO, person only)
    mapping              SLAM Toolbox -> husky1/map
    path planning        Nav2, goals set from RViz
    hazard response      hazard_brake cancels navigation on a close person

    ros2 launch 41068_ignition_bringup ammo_demo.launch.py

What to film, in order:

  1. Detection. Drive with the teleop window past mushrooms and people and
     watch husky1/combined/image - green boxes are mushrooms, red are people,
     with a running count in the corner. Nothing else needs to be running.
  2. Path planning. Relaunch with guard:=False (see below), wait for the map
     to build, then use RViz's "2D Goal Pose" button to send the Husky
     somewhere. The green line is the global plan.
  3. Hazard stop. Send a goal with a person between the robot and the goal.
     hazard_brake cancels the goal as the person fills ~120 px of frame, and
     prints HAZARD to the console. `ros2 topic echo /husky1/hazard_stop`
     shows the state flipping if you want it on camera.

Film the teleop clips and the Nav2 clips as separate runs. Teleop reaches the
robot through collision_guard (guard:=True, the default), which keeps
publishing cmd_vel once it has seen a single command - so it would fight Nav2's
velocity smoother over the same topic. For clips 2 and 3:

    ros2 launch 41068_ignition_bringup ammo_demo.launch.py guard:=False teleop:=False

Wiring Nav2's output through the guard instead of around it is the right fix,
and is what a merged pipeline should do - but it changes the shared navigation
launch file, so it is left out of a demo-only change.
"""
import math
import os
import random

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# People go further out than mushrooms: the point is to see one coming, not to
# have it spawn on top of the robot.
SCATTER = [('mushroom_amanita', 'mushrooms', 0.0, 3.0, 9.0),
           ('walking_person', 'people', 0.0, 4.0, 10.0),
           ('decoy_box', None, 0.3, 3.0, 9.0)]
SPAWN_DELAY_S = 10.0


def scatter_nodes(context, use_sim_time):
    nodes = []
    for model, arg, z, r_min, r_max in SCATTER:
        count = int(LaunchConfiguration(arg).perform(context)) if arg else 3
        for i in range(count):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(r_min, r_max)
            nodes.append(Node(
                package='ros_ign_gazebo', executable='create',
                name=f'spawn_{model}_{i}', output='screen',
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
    # with an incompatible libpthread, crashing any Qt/GTK app started from an
    # integrated terminal. Strip them for children.
    for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
                'GIO_MODULE_DIR', 'LOCPATH', 'GSETTINGS_SCHEMA_DIR'):
        os.environ.pop(var, None)

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')

    for name, default, desc in (
        ('use_sim_time', 'True', 'Use simulation clock'),
        ('world', 'simple_trees', 'Which world to load'),
        ('nav2', 'True', 'Run SLAM and Nav2 so goals can be set in RViz'),
        ('teleop', 'True', 'Open an xterm for driving by hand'),
        ('mushrooms', '6', 'How many mushrooms to scatter'),
        ('people', '3', 'How many people to scatter'),
        ('conf_threshold', '0.6',
         'Mushroom detector threshold; raise it if people draw false '
         'mushroom boxes, lower it if distant mushrooms are missed'),
        ('stop_height_px', '120.0',
         'Person height in frame that triggers the hazard stop'),
        ('guard', 'True',
         'Run collision_guard so teleop commands pass through the lidar '
         'safety gate. Turn it OFF before sending Nav2 goals: once the guard '
         'has seen a teleop command it keeps publishing cmd_vel forever, and '
         'two publishers on that topic fight over the robot.'),
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
            'husky': 'True',
            'parrot': 'False',
            'rviz': 'True',
            'slam': LaunchConfiguration('nav2'),
            'nav2': LaunchConfiguration('nav2'),
            'yolo': 'True',
            'hazard': 'True',
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'hazard_conf': '0.65',
            'stop_height_px': LaunchConfiguration('stop_height_px'),
        }.items(),
    ))

    # Keyboard teleop needs a real terminal for stdin.
    ld.add_action(TimerAction(period=8.0, actions=[ExecuteProcess(
        cmd=['xterm', '-title', 'husky1 teleop', '-geometry', '80x28', '-e',
             'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
             '--ros-args', '-r', 'cmd_vel:=/husky1/cmd_vel_raw',   # via the guard
             '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')))]))

    # Lidar safety gate: cmd_vel_raw -> cmd_vel. Without it nothing consumes
    # what teleop publishes and the robot will not move at all.
    ld.add_action(TimerAction(period=7.0, actions=[Node(
        package='skidsteer_controller', executable='collision_guard_node',
        name='collision_guard', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # the depth check reads the camera pose from husky1's TF tree
        remappings=[('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')],
        condition=IfCondition(LaunchConfiguration('guard')))]))

    # One view with both detectors drawn on it.
    ld.add_action(TimerAction(period=12.0, actions=[Node(
        package='rqt_image_view', executable='rqt_image_view',
        name='view_combined', output='screen',
        arguments=['/husky1/combined/image'],
        parameters=[{'use_sim_time': use_sim_time}])]))

    ld.add_action(TimerAction(period=SPAWN_DELAY_S, actions=[
        OpaqueFunction(function=lambda ctx: scatter_nodes(ctx, use_sim_time))]))

    return ld
