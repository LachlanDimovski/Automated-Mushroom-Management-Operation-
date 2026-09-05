"""Launches the objective-trail dashboard variant (see objective_trail_dashboard.py).

Runs the node via `python3 -m` instead of a console_script so nothing needs
to be registered in setup.py - keeps this feature isolated from the base
dashboard entry point.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_range_m = LaunchConfiguration('map_range_m')
    fps = LaunchConfiguration('fps')
    delay_s = LaunchConfiguration('delay_s')
    triangle_button = LaunchConfiguration('triangle_button')
    r1_button = LaunchConfiguration('r1_button')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument('map_range_m', default_value='36.0'))
    ld.add_action(DeclareLaunchArgument('fps', default_value='12.0'))
    # Keep in sync with ammo_mission_control's own delay_s.
    ld.add_action(DeclareLaunchArgument('delay_s', default_value='10.0'))
    # Must match mushroom_mission_node's own triangle_button/r1_button.
    ld.add_action(DeclareLaunchArgument('triangle_button', default_value='2'))
    ld.add_action(DeclareLaunchArgument('r1_button', default_value='5'))

    ld.add_action(ExecuteProcess(
        cmd=[
            'python3', '-m', 'ammo_dashboard.objective_trail_dashboard',
            '--ros-args',
            '-r', '__node:=ammo_dashboard_objective_trail',
            '-p', ['use_sim_time:=', use_sim_time],
            '-p', ['map_range_m:=', map_range_m],
            '-p', ['fps:=', fps],
            '-p', ['delay_s:=', delay_s],
            '-p', ['triangle_button:=', triangle_button],
            '-p', ['r1_button:=', r1_button],
        ],
        output='screen',
    ))

    return ld
