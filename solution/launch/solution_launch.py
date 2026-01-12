import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap, PushRosNamespace, RosTimer, SetUseSimTime


def robot_controller_actions(context: LaunchContext):
    num_robots = int(context.launch_configurations['num_robots'])

    initial_pose_pkg = context.launch_configurations['initial_pose_package']
    initial_pose_file = context.launch_configurations['initial_pose_file']

    pkg_share = get_package_share_directory(initial_pose_pkg)
    yaml_path = os.path.join(pkg_share, initial_pose_file)

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f) or {}

    poses_block = (
        configuration.get(num_robots)
        or configuration.get(str(num_robots))
        or configuration
    )

    actions = []

    for robot_number in range(1, num_robots + 1):
        robot_name = f'robot{robot_number}'

        robot_params = poses_block.get(robot_name, {}) if isinstance(poses_block, dict) else {}

        group = GroupAction([
            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                output='screen',
                parameters=[robot_params],
            ),
        ])

        actions.append(group)

    return actions


def generate_launch_description():
    package_name = 'solution'

    num_robots = LaunchConfiguration('num_robots')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')
    data_log_path = LaunchConfiguration('data_log_path')
    data_log_filename = LaunchConfiguration('data_log_filename')
    obstacles = LaunchConfiguration('obstacles')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_log_level = LaunchConfiguration('rviz_log_level')
    sensor_noise = LaunchConfiguration('sensor_noise')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    odometry_source = LaunchConfiguration('odometry_source')
    barrel_manager = LaunchConfiguration('barrels')
    headless = LaunchConfiguration('headless')
    limit_real_time_factor = LaunchConfiguration('limit_real_time_factor')
    wait_for_barrels = LaunchConfiguration('wait_for_barrels')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose')

    vision_sensor_skip_frames = LaunchConfiguration('vision_sensor_skip_frames')
    vision_sensor_frame_divider = LaunchConfiguration('vision_sensor_frame_divider')
    vision_sensor_debug = LaunchConfiguration('vision_sensor_debug')

    initial_pose_package = LaunchConfiguration('initial_pose_package')
    initial_pose_file = LaunchConfiguration('initial_pose_file')

    # ---- Declare args (must retain assessment-supported params) ----
    ld = LaunchDescription()
    ld.add_action(SetUseSimTime(True))

    ld.add_action(DeclareLaunchArgument(
        'data_log_path',
        default_value=os.path.join(get_package_prefix(package_name), '../../'),
        description='Full path to directory where data logs will be saved'))

    ld.add_action(DeclareLaunchArgument(
        'data_log_filename',
        default_value='data_log',
        description='Filename prefix to use for data logs'))

    ld.add_action(DeclareLaunchArgument('use_nav2', default_value='False'))
    ld.add_action(DeclareLaunchArgument('barrels', default_value='True'))
    ld.add_action(DeclareLaunchArgument('wait_for_barrels', default_value='True'))
    ld.add_action(DeclareLaunchArgument('visual_sensor', default_value='True'))
    ld.add_action(DeclareLaunchArgument('vision_sensor_skip_frames', default_value='True'))
    ld.add_action(DeclareLaunchArgument('vision_sensor_debug', default_value='False'))
    ld.add_action(DeclareLaunchArgument('vision_sensor_frame_divider', default_value='0.5'))

    ld.add_action(DeclareLaunchArgument('obstacles', default_value='true'))
    ld.add_action(DeclareLaunchArgument('num_robots', default_value='1'))
    ld.add_action(DeclareLaunchArgument('random_seed', default_value='0'))
    ld.add_action(DeclareLaunchArgument('experiment_duration', default_value='840.0'))

    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz_log_level', default_value='warn'))
    ld.add_action(DeclareLaunchArgument('sensor_noise', default_value='False'))
    ld.add_action(DeclareLaunchArgument('visualise_sensors', default_value='false'))
    ld.add_action(DeclareLaunchArgument('odometry_source', default_value='ENCODER'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='False'))
    ld.add_action(DeclareLaunchArgument('limit_real_time_factor', default_value='True'))
    ld.add_action(DeclareLaunchArgument('gazebo_verbose', default_value='False'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pose_file',
        default_value='config/initial_poses.yaml',
        description="Location of initial pose yaml file relative to the package in 'initial_pose_package'"))

    ld.add_action(DeclareLaunchArgument(
        'initial_pose_package',
        default_value='assessment',
        description="Package name for finding the file 'config/initial_poses.yaml'"))

    # ---- Paths used by assessment launch ----
    map_yaml = PathJoinSubstitution([FindPackageShare('solution'), 'config', 'map2.yaml'])
    nav2_params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('assessment'), 'launch', 'assessment.launch.py'])
        ),
        launch_arguments={
            'num_robots': num_robots,
            'visualise_sensors': visualise_sensors,
            'odometry_source': odometry_source,
            'sensor_noise': sensor_noise,
            'use_rviz': use_rviz,
            'barrels': barrel_manager,
            'random_seed': random_seed,
            'use_nav2': use_nav2,
            'map': map_yaml,
            'params_files': nav2_params,
            'headless': headless,
            'obstacles': obstacles,
            'limit_real_time_factor': limit_real_time_factor,
            'wait_for_barrels': wait_for_barrels,
            'initial_pose_file': initial_pose_file,
            'initial_pose_package': initial_pose_package,
            'gazebo_verbose': gazebo_verbose,
            'rviz_log_level': rviz_log_level
        }.items()
    )

    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    data_logger_cmd = Node(
        package='solution',
        executable='data_logger',
        output='screen',
        arguments=['--path', data_log_path,
                   '--filename', data_log_filename,
                   '--random_seed', random_seed]
    )

    timeout_cmd = RosTimer(
        period=experiment_duration,
        actions=[Shutdown(reason="Experiment timeout reached")]
    )

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(data_logger_cmd)
    ld.add_action(timeout_cmd)

    return ld
