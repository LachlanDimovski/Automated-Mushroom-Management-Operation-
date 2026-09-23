
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('41068_ignition_bringup')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # Start the main Gazebo simulation with only the robots.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg,
                'launch',
                '41068_ignition.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'headless': headless,

            # Robots
            'husky': 'true',
            'parrot': 'true',

            # Disable navigation/perception/UI extras
            'nav2': 'false',
            'slam': 'false',
            'yolo': 'false',
            'rviz': 'false',
            'ground_truth_localization': 'false',
        }.items(),
    )

    # Spawn mushrooms after Gazebo and the robots have loaded.
    mushroom_sdf = PathJoinSubstitution([
        pkg,
        'models',
        'mushroom_amanita',
        'model.sdf'
    ])

    mushroom_positions = [
        (4.0, 2.0),
        (3.0, -3.0),
        (-2.0, 3.0),
        (-4.0, -2.0),
        (1.0, 5.0),
        (-5.0, 1.0),
    ]

    mushroom_spawns = []

    for i, (x, y) in enumerate(mushroom_positions):
        mushroom_spawns.append(
            Node(
                package='ros_ign_gazebo',
                executable='create',
                name=f'spawn_photo_mushroom_{i}',
                output='screen',
                arguments=[
                    '-file', mushroom_sdf,
                    '-name', f'photo_mushroom_{i}',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.0',
                ],
            )
        )

    spawn_mushrooms = TimerAction(
        period=8.0,
        actions=mushroom_spawns,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simple_trees',
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='false',
        ),

        simulation,
        spawn_mushrooms,
    ])

