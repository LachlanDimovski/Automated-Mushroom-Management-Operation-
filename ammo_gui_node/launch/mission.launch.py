"""The mission nodes, without the simulation.

    ros2 launch ammo_gui_node mission.launch.py world:=simple_trees

The same node set and parameters as `ammo_mission_control/cage_mission.launch.py`,
minus its `forest_world.launch.py` include - so it can be run against a
simulation that is already up instead of starting a second one.

The GUI does not use this file (it starts each node itself so it can pause
them individually). It is here for running a mission without the GUI, and as
the readable version of what the GUI is doing behind the button.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from ammo_gui_node.mission_launcher import world_meta
    world = LaunchConfiguration('world').perform(context)
    tool = LaunchConfiguration('tool').perform(context)
    with_quad = LaunchConfiguration('parrot').perform(context).lower() == 'true'
    trunks, ground_z, half = world_meta(world)
    sim = {'use_sim_time': True}

    nodes = [
        Node(package='ammo_mission_control', executable='truth_localizer', output='screen',
             parameters=[sim],
             remappings=[('/tf', '/husky1/tf'), ('/tf_static', '/husky1/tf_static')]),
        Node(package='skidsteer_controller', executable='collision_guard_node',
             output='screen', parameters=[sim]),
        Node(package='ammo_mission_control', executable='cage_capture_node', output='screen',
             parameters=[{**sim, 'world': world, 'trunks': trunks,
                          'count': 0, 'tool': tool}]),
    ]
    if with_quad:
        alt = 4.0 if half < 10 else 6.0
        nodes.insert(2, Node(
            package='ammo_mission_control', executable='quad_search_node', output='screen',
            parameters=[{**sim, 'trunks': trunks, 'waypoints': [0.0], 'ground_z': ground_z,
                         'search_area': [-half, half, -half, half, alt]}]))
    return nodes


def generate_launch_description():
    ld = LaunchDescription()
    for name, default in (('world', 'simple_trees'), ('tool', 'gripper'), ('parrot', 'true')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default))
    ld.add_action(OpaqueFunction(function=_nodes))
    return ld
