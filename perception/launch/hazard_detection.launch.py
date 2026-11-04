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
    # One core to itself, like the mission detector. yolo_node pins the whole
    # process before any thread pool spawns, and its own comment measures the
    # difference as 3.2 s/frame unpinned against 0.3 s/frame pinned - so a
    # second detector left floating does not just cost its own core, it takes
    # time off the one that was pinned. '' = unpinned.
    ld.add_action(DeclareLaunchArgument('core', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'stop_height_px', default_value='180.0',
        description='All-round backstop: a person whose box is this tall '
                    'stops the rover wherever they are in frame. 180 px is '
                    '2.0 m on this camera. The directional test - a person '
                    'within 3 m AND inside the rover\'s corridor - is what '
                    'normally fires first; see hazard_brake.py.'))
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
        additional_env={'YOLO_CORE': LaunchConfiguration('core')},
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
