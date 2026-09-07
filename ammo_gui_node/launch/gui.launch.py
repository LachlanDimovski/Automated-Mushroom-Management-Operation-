"""Launches the operator GUI on its own - run instead of ammo_dashboard,
not alongside it (they're two independent windows, nothing stops running
both, but they're not meant to replace each other's job).

    ros2 launch ammo_gui_node gui.launch.py

Needs the core sim already up (see 41068_ignition_bringup in the cheat
sheet) - the GUI itself launches whichever ammo_mission_control launch file
matches the mode you pick on its home page (see mission_launcher.py), but it
does not start Gazebo/SLAM/Nav2/the robots themselves.

husky_spawn/parrot_spawn follow the same convention ammo_dashboard and
ammo_mission_control already use: husky1/odometry and parrot1/odometry are
raw world-absolute Gazebo bridge topics, so these two params are just where
each robot's own odom origin sits in that shared world frame (spawn point
from 41068_ignition.launch.py's add_robot calls), used to line the map,
robot markers and mushroom markers up on one shared canvas.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    map_topic = LaunchConfiguration('map_topic')
    mushroom_found_topic = LaunchConfiguration('mushroom_found_topic')
    teleop_topic = LaunchConfiguration('teleop_topic')

    # husky1/map (not parrot1/map): ground-based SLAM is denser/more
    # reliable than the quad's airborne SLAM, and it's the frame the husky
    # actually navigates in - see the map-source discussion in the repo
    # docs. Override at launch if you want the quad's map instead.
    ld.add_action(DeclareLaunchArgument('map_topic', default_value='husky1/map'))
    ld.add_action(DeclareLaunchArgument(
        'mushroom_found_topic', default_value='mushroom_found'))
    ld.add_action(DeclareLaunchArgument(
        'teleop_topic', default_value='/parrot1/cmd_vel'))

    # husky_spawn/parrot_spawn are not exposed as launch args, matching
    # ammo_dashboard/mushroom_mission_node - they're the same fixed spawn
    # points from 41068_ignition.launch.py's add_robot calls, not something
    # anyone currently overrides at launch time.
    ld.add_action(Node(
        package='ammo_gui_node',
        executable='gui',
        name='ammo_gui_node',
        output='screen',
        parameters=[{
            'map_topic': map_topic,
            'mushroom_found_topic': mushroom_found_topic,
            'teleop_topic': teleop_topic,
            'husky_spawn': [0.0, 0.0],
            'parrot_spawn': [2.0, 0.0],
        }],
    ))

    return ld
