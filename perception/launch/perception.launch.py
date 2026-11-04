"""Perception stack for one robot.

Every bringup that wants detections includes this file with a `robot`
namespace. Add future perception nodes (LiDAR, etc.) here so they come up
everywhere at once.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    ld.add_action(DeclareLaunchArgument('hazard', default_value='False'))
    ld.add_action(DeclareLaunchArgument('hazard_conf', default_value='0.65'))
    ld.add_action(DeclareLaunchArgument('stop_height_px', default_value='120.0'))
    # one core per detector (see yolo_node.py); '' = unpinned
    ld.add_action(DeclareLaunchArgument('core', default_value=''))
    # A different one from `core`, or the two detectors on the rover land on
    # the same logical CPU and queue behind each other.
    ld.add_action(DeclareLaunchArgument('hazard_core', default_value=''))
    # combined_view draws both detectors onto one picture. Off by default
    # now: the operator GUI already merges the mushroom boxes and the person
    # boxes onto the rover's own camera tile, so on a GUI run this is a third
    # node decoding the same images at camera rate for a window nobody has
    # open. Pass combined_view:=true for a standalone recording.
    ld.add_action(DeclareLaunchArgument('combined_view', default_value='False'))
    ld.add_action(DeclareLaunchArgument('imgsz', default_value='384'))  # 640 keeps small, far mushrooms detectable

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
            'imgsz': ParameterValue(LaunchConfiguration('imgsz'), value_type=int),
        }],
        condition=IfCondition(yolo),
        additional_env={'YOLO_CORE': LaunchConfiguration('core')},
    ))

    # Person detection, on stock COCO weights. Separate file because its
    # model, threshold and topics have nothing in common with the mushroom
    # detector's - see hazard_detection.launch.py for why it is a second node
    # rather than a second class.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('perception'),
            'launch', 'hazard_detection.launch.py',
        ])),
        launch_arguments={
            'robot': robot,
            'use_sim_time': use_sim_time,
            'hazard': LaunchConfiguration('hazard'),
            'conf_threshold': LaunchConfiguration('hazard_conf'),
            'stop_height_px': LaunchConfiguration('stop_height_px'),
            'core': LaunchConfiguration('hazard_core'),
        }.items(),
    ))

    # A single view with both detectors drawn on it, so one recording covers
    # targets and hazards. Only worth running when both are up.
    ld.add_action(Node(
        package='perception',
        executable='combined_view',
        namespace=robot,
        name='combined_view',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'image_topic': 'camera/image'}],
        condition=IfCondition(PythonExpression(
            ["'", yolo, "'.lower() in ['true','1','yes','on'] and '",
             LaunchConfiguration('hazard'),
             "'.lower() in ['true','1','yes','on'] and '",
             LaunchConfiguration('combined_view'),
             "'.lower() in ['true','1','yes','on']"])),
    ))

    return ld
