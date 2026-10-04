from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_STRINGS = "['true', '1', 'yes', 'on']"


def _is_true_expression(name):
    return ["'", LaunchConfiguration(name), "'.lower() in ", _TRUE_STRINGS]


def _if_true(name):
    return IfCondition(PythonExpression(_is_true_expression(name)))


def _if_all_true(*names):
    pieces = []
    for i, name in enumerate(names):
        if i > 0:
            pieces.append(' and ')
        pieces.extend(_is_true_expression(name))
    return IfCondition(PythonExpression(pieces))


def _if_any_of_and(any_of, *names):
    """IfCondition for (any of `any_of`) and (all of `names`).

    Perception has two independent detectors but one include, so it must come
    up when either is wanted - while still respecting whether the robot itself
    is enabled.
    """
    pieces = ['(']
    for i, name in enumerate(any_of):
        if i > 0:
            pieces.append(' or ')
        pieces.extend(_is_true_expression(name))
    pieces.append(')')
    for name in names:
        pieces.append(' and ')
        pieces.extend(_is_true_expression(name))
    return IfCondition(PythonExpression(pieces))


def add_robot(
    ld,
    *,
    pkg_path,
    config_path,
    use_sim_time,
    enabled_arg,
    namespace,
    frame_prefix,
    gz_model_name,
    xacro_parts,
    bridge_config,
    localization_config,
    x,
    y,
    z,
    yaw='0.0',
    spawn_delay=0.0,
):
    """Add one robot instance to the launch description.

    Convention used here:
      - ROS nodes/topics are under /<namespace>/...
      - Gazebo entity/model is named <gz_model_name>
      - Gazebo plugin topics are under /model/<gz_model_name>/...
      - TF topics are under /<namespace>/tf and /<namespace>/tf_static
      - TF frame ids use <frame_prefix>, e.g. husky1_base_link
    """

    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path] + xacro_parts),
            ' ',
            'prefix:=', frame_prefix,
            ' ',
            'gz_model_name:=', gz_model_name,
        ]),
        value_type=str,
    )

    robot_enabled = _if_true(enabled_arg)

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, localization_config]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
        ],
        condition=robot_enabled,
    ))

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_' + namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/' + namespace + '/robot_description',
            '-name', gz_model_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    # Start Gazebo first, then insert robot entities in a predictable order.
    ld.add_action(TimerAction(
        period=spawn_delay,
        actions=[robot_spawner],
        condition=robot_enabled,
    ))

    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, bridge_config]),
            'use_sim_time': use_sim_time,
        }],
        condition=robot_enabled,
    ))


def add_navigation_instance(
    ld,
    *,
    pkg_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    start_delay,
):
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_path,
                'launch',
                '41068_navigation.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': robot_namespace,
            'config_filename_suffix': '_' + robot_namespace,
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
            'ground_truth_localization': LaunchConfiguration('ground_truth_localization'),
        }.items(),
    )

    # Scope is important: without it, repeated IncludeLaunchDescription calls can
    # leak/overwrite launch configurations such as namespace and config suffix.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[GroupAction(
            scoped=True,
            actions=[nav_include],
        )],
        condition=_if_true(robot_arg),
    ))


def add_perception(
    ld,
    *,
    use_sim_time,
    robot_arg,
    robot_namespace,
    start_delay,
    core='',
    yolo_arg='yolo',
):
    """Run the shared perception stack (YOLO detector) for one robot.

    Publishes <namespace>/detections and <namespace>/detections/image.
    """
    perception_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'perception.launch.py',
            ])
        ),
        launch_arguments={
            'robot': robot_namespace,
            'use_sim_time': use_sim_time,
            'yolo': LaunchConfiguration(yolo_arg),
            'hazard': LaunchConfiguration('hazard'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'hazard_conf': LaunchConfiguration('hazard_conf'),
            'stop_height_px': LaunchConfiguration('stop_height_px'),
            'core': core,
        }.items(),
    )

    ld.add_action(TimerAction(
        period=start_delay,
        actions=[GroupAction(scoped=True, actions=[perception_include])],
        # Either detector is reason enough to bring perception up; which
        # nodes actually run is decided inside perception.launch.py.
        condition=_if_any_of_and((yolo_arg, 'hazard'), robot_arg),
    ))


def add_rviz_instance(
    ld,
    *,
    config_path,
    use_sim_time,
    robot_arg,
    robot_namespace,
    rviz_config,
    start_delay,
):
    # Each RViz process is itself placed in the robot namespace so the Nav2 RViz
    # panel resolves relative action/service names such as navigate_to_pose and
    # lifecycle_manager_navigation to the correct robot.
    ld.add_action(TimerAction(
        period=start_delay,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_namespace,
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([config_path, rviz_config])],
            remappings=[
                ('/tf', '/' + robot_namespace + '/tf'),
                ('/tf_static', '/' + robot_namespace + '/tf_static'),
            ],
        )],
        condition=_if_all_true('rviz', robot_arg),
    ))


def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz',
    )
    ld.add_action(rviz_launch_arg)

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Flag to launch SLAM Toolbox for each enabled robot',
    )
    ld.add_action(slam_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2 for each enabled robot. Nav2 also starts SLAM.',
    )
    ld.add_action(nav2_launch_arg)
    ld.add_action(DeclareLaunchArgument(
        'ground_truth_localization', default_value='False',
        description='Skip SLAM; map->odom comes from an external ground-truth localizer (sim only)'))

    husky_launch_arg = DeclareLaunchArgument(
        'husky',
        default_value='True',
        description='Launch the Husky UGV instance in namespace /husky1',
    )
    ld.add_action(husky_launch_arg)

    parrot_launch_arg = DeclareLaunchArgument(
        'parrot',
        default_value='False',
        description='Launch the Parrot drone instance in namespace /parrot1',
    )
    ld.add_action(parrot_launch_arg)

    ld.add_action(DeclareLaunchArgument(
        'conf_threshold', default_value='0.6',
        description='Confidence threshold for the mushroom detector. Held '
                    'well above the usual 0.25 because the model was trained '
                    'only on scenes containing mushrooms - anything it has '
                    'never seen, a person especially, draws low-confidence '
                    'false positives. Real mushrooms score above 0.9, so this '
                    'costs about 1% of true detections.'))
    ld.add_action(DeclareLaunchArgument(
        'hazard_conf', default_value='0.65',
        description='Confidence threshold for the person detector'))
    ld.add_action(DeclareLaunchArgument(
        'stop_height_px', default_value='120.0',
        description='Person height in frame that triggers the hazard stop'))
    ld.add_action(DeclareLaunchArgument(
        'hazard',
        default_value='False',
        description='Launch the COCO person detector for hazard awareness. '
                    'Off by default: it is a second inference pass per camera '
                    '(~90 ms of a 333 ms budget) and the mission path does not '
                    'need it.',
    ))

    yolo_launch_arg = DeclareLaunchArgument(
        'yolo',
        default_value='True',
        description='Run the shared YOLO detector for each enabled robot',
    )
    ld.add_action(yolo_launch_arg)
    ld.add_action(DeclareLaunchArgument(
        'husky_yolo', default_value='True',
        description='Also required for the husky detector (the mission approach is odometry-only and does not need it)'))
    # husky detector only when both yolo and husky_yolo are true
    ld.add_action(SetLaunchConfiguration('husky_yolo_on', PythonExpression(
        _is_true_expression('yolo') + [' and '] + _is_true_expression('husky_yolo'))))

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load: any worlds/<name>.sdf (simple_trees, large_demo, forest_flat, forest_random, ...)',
    )
    ld.add_action(world_launch_arg)

    headless_launch_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo server-only (no GUI window); use RViz for visualization instead',
    )
    ld.add_action(headless_launch_arg)

    # Load common Gazebo server systems from a shared config file. This keeps
    # required systems such as Sensors out of individual robot and world files,
    # so custom student worlds do not need to copy plugin blocks.
    server_config_file = PathJoinSubstitution([
        config_path,
        'ignition_server.config',
    ])
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))
    ld.add_action(SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=server_config_file,
    ))

    # Start Gazebo once.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([
                    pkg_path,
                    'worlds',
                    [LaunchConfiguration('world'), '.sdf'],
                ]),
                PythonExpression([
                    "' -s -r' if '", LaunchConfiguration('headless'),
                    "'.lower() in ", "['true', '1', 'yes', 'on']", " else ' -r'",
                ]),
            ]
        }.items(),
    ))

    # Bridge the global simulation clock once.
    ld.add_action(Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge_clock.yaml']),
            'use_sim_time': use_sim_time,
        }],
    ))

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='husky',
        namespace='husky1',
        frame_prefix='husky1_',
        gz_model_name='husky1',
        xacro_parts=['urdf_husky', 'husky.urdf.xacro'],
        bridge_config='gazebo_bridge_husky1.yaml',
        localization_config='robot_localization_husky1.yaml',
        x='0.0',
        y='0.0',
        z='0.4',
        spawn_delay=3.0,
    )

    add_robot(
        ld,
        pkg_path=pkg_path,
        config_path=config_path,
        use_sim_time=use_sim_time,
        enabled_arg='parrot',
        namespace='parrot1',
        frame_prefix='parrot1_',
        gz_model_name='parrot1',
        xacro_parts=['urdf_parrot', 'parrot.urdf.xacro'],
        bridge_config='gazebo_bridge_parrot1.yaml',
        localization_config='robot_localization_parrot1.yaml',
        x='2.0',
        y='0.0',
        z='0.8',
        spawn_delay=6.0,
    )

    add_navigation_instance(
        ld,
        pkg_path=pkg_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        start_delay=7.0,
    )

    # one CPU core per detector (see perception/yolo_node.py)
    add_perception(
        ld,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        start_delay=8.0,
        core='6',
        yolo_arg='husky_yolo_on',
    )

    add_perception(
        ld,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        start_delay=11.0,
        core='7',
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='husky',
        robot_namespace='husky1',
        rviz_config='41068_husky1.rviz',
        start_delay=11.0,
    )

    add_rviz_instance(
        ld,
        config_path=config_path,
        use_sim_time=use_sim_time,
        robot_arg='parrot',
        robot_namespace='parrot1',
        rviz_config='41068_parrot1.rviz',
        start_delay=13.0,
    )

    return ld
