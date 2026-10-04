"""Person detection for hazard awareness, on one robot.

Runs a second detector alongside the mushroom one, using stock COCO weights
rather than the fine-tuned model. Two reasons it is a separate node and not a
class in the mission model:

  * COCO's `person` is trained on ~250k real instances. Nothing we could
    capture in Gazebo would beat that, and it costs no training time.
  * The mushroom model is single-class - fine-tuning replaced the detection
    head, so those 80 COCO classes no longer exist in it.

The cost is one extra inference pass per camera, measured at ~90 ms against a
333 ms budget at the 3 Hz camera rate. Fold `person` into the mission model
only if that stops fitting on the slowest machine in the team.

    ros2 launch perception hazard_detection.launch.py robot:=husky1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

COCO_WEIGHTS = os.path.join(
    get_package_share_directory('perception'), 'models', 'yolo11n_coco.pt')
PERSON_CLASS = 0        # COCO class index


def generate_launch_description():
    ld = LaunchDescription()

    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hazard = LaunchConfiguration('hazard')
    conf_threshold = LaunchConfiguration('conf_threshold')

    ld.add_action(DeclareLaunchArgument('robot', default_value='husky1'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('hazard', default_value='True'))
    ld.add_action(DeclareLaunchArgument(
        'stop_height_px', default_value='120.0',
        description='A person taller than this in frame triggers a stop. '
                    'Height is a stand-in for range: 120 px is roughly 3 m '
                    'for a 1.9 m person on this camera.'))
    ld.add_action(DeclareLaunchArgument(
        'conf_threshold', default_value='0.65',
        description='Higher than the mushroom detector on purpose: at low '
                    'confidence the COCO model calls mushrooms people. It '
                    'detects reliably up close, which is the range that '
                    'matters for stopping.'))

    ld.add_action(Node(
        package='perception',
        executable='yolo_node',
        namespace=robot,
        name='hazard_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': 'camera/image',
            'model_path': COCO_WEIGHTS,
            'classes': [PERSON_CLASS],
            'conf_threshold': ParameterValue(conf_threshold, value_type=float),
        }],
        # Its own topics: the mission detector owns `detections`, and mixing
        # two different class sets on one topic would be silently wrong.
        remappings=[('detections', 'hazards'),
                    ('detections/image', 'hazards/image')],
        condition=IfCondition(hazard),
    ))

    # Acts on those detections: cancels navigation when a person is close.
    # Harmless with Nav2 absent - it just logs that the service is missing.
    ld.add_action(Node(
        package='perception',
        executable='hazard_brake',
        namespace=robot,
        name='hazard_brake',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'stop_height_px': LaunchConfiguration('stop_height_px'),
        }],
        condition=IfCondition(hazard),
    ))

    return ld
