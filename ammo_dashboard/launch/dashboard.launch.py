from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_range_m = LaunchConfiguration('map_range_m')
    fps = LaunchConfiguration('fps')

    # The HUD shows live data and times staleness off the wall clock, so it
    # runs on real time regardless of how fast the sim is stepping.
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument('map_range_m', default_value='12.0'))
    ld.add_action(DeclareLaunchArgument('fps', default_value='12.0'))

    ld.add_action(Node(
        package='ammo_dashboard',
        executable='dashboard',
        name='ammo_dashboard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_range_m': ParameterValue(map_range_m, value_type=float),
            'fps': ParameterValue(fps, value_type=float),
        }],
    ))

    return ld
