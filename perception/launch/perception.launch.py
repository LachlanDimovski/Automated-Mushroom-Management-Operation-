"""Perception stack for one robot.

Every bringup that wants detections includes this file with a `robot`
namespace. Add future perception nodes (LiDAR, etc.) here so they come up
everywhere at once.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    yolo = LaunchConfiguration('yolo')
    model_path = LaunchConfiguration('model_path')
    conf_threshold = LaunchConfiguration('conf_threshold')

    ld.add_action(DeclareLaunchArgument('robot', default_value='husky1'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('yolo', default_value='True'))
    ld.add_action(DeclareLaunchArgument('model_path', default_value=''))
    ld.add_action(DeclareLaunchArgument('conf_threshold', default_value='0.25'))

    ld.add_action(Node(
        package='perception',
        executable='yolo_node',
        namespace=robot,
        name='yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': 'camera/image',
            'model_path': model_path,
            'conf_threshold': ParameterValue(conf_threshold, value_type=float),
        }],
        condition=IfCondition(yolo),
    ))

    return ld
