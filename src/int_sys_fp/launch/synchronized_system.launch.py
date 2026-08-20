#!/usr/bin/env python3

import os
import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, NotSubstitution, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file per sistema sincronizzato: Gazebo (empty_world) + 3 TurtleBot3 + UWB Emulator + Controller + KF
    """

    # Basic launch arguments
    uwb_frequency_arg = DeclareLaunchArgument(
        'uwb_frequency',
        default_value='50.0',
        description='UWB sensor frequency in Hz'
    )
    
    noise_type_arg = DeclareLaunchArgument(
        'noise_type',
        default_value='1',
        description='Noise type for UWB sensor: 1=Gaussian, 2=Uniform'
    )
    
    enable_planner_arg = DeclareLaunchArgument(
        'enable_planner',
        default_value='true',
        description='Enable trajectory planner node (circular trajectory)'
    )
    
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='ekf',
        description="Pose filter to use for all 3 robots: 'ekf' or 'ukf'"
    )

    enable_legacy_kf_arg = DeclareLaunchArgument(
        'enable_legacy_kf',
        default_value='false',
        description='Also launch the legacy distance-space C++ Kalman Filter (baseline for comparison, not used by the controllers)'
    )

    enable_bag_record_arg = DeclareLaunchArgument(
        'enable_bag_record',
        default_value='true',
        description='Record a ros2 bag of all estimation/control topics for offline (e.g. Matlab) analysis'
    )

    enable_plotjuggler_arg = DeclareLaunchArgument(
        'enable_plotjuggler',
        default_value='true',
        description='Launch PlotJuggler for live topic visualization (requires ros-humble-plotjuggler-ros)'
    )

    # --- Bag recording setup ---
    # Recorded under ~/ros2_ws/bags/ (workspace root, NOT install/) so bags survive
    # a `rm -rf install/` clean rebuild (see README troubleshooting).
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_output_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'bags', f'int_sys_sim_{timestamp}')

    bag_topics = [
        '/clock',
        '/uwb/anchor_distances', '/uwb/robot_distances',
        '/uwb/filtered_anchor_distances', '/uwb/filtered_robot_distances',
        '/odom', '/tb3_2/odom', '/tb3_3/odom',
        '/imu', '/tb3_2/imu', '/tb3_3/imu',
        '/cmd_vel', '/tb3_2/cmd_vel', '/tb3_3/cmd_vel',
        '/pose_estimate', '/tb3_2/pose_estimate', '/tb3_3/pose_estimate',
        '/pose_debug', '/tb3_2/pose_debug', '/tb3_3/pose_debug',
        '/fsm_state', '/tb3_2/fsm_state', '/tb3_3/fsm_state',
        '/tracking_error', '/tb3_2/tracking_error', '/tb3_3/tracking_error',
        '/centroid_position', '/tb3_2/centroid_position', '/tb3_3/centroid_position',
        '/desired_trajectory', '/desired_trajectory_array',
        '/tb3_2/desired_trajectory_array', '/tb3_3/desired_trajectory_array',
    ]

    create_bags_dir = ExecuteProcess(
        cmd=['mkdir', '-p', os.path.dirname(bag_output_dir)],
        output='screen',
        shell=False
    )

    bag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_output_dir] + bag_topics,
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bag_record'))
    )

    # Start recording once robots are spawned and estimators/controllers are up
    # (last spawn fires at 13s) rather than from t=0.
    delayed_bag_record = TimerAction(period=16.0, actions=[bag_record_process])

    # --- PlotJuggler (live visualization) ---
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_plotjuggler'))
    )

    # Ensure TurtleBot3 model env var is set for gazebo/model resolution
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    set_gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', '/opt/ros/humble/lib']
    )

    # Include Gazebo without any robot pre-spawned
    # We use gazebo_ros package directly instead of empty_world to avoid auto-spawning
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tb3_gz_share = get_package_share_directory('turtlebot3_gazebo')
    
    # Get the world file path
    world_file = os.path.join(tb3_gz_share, 'worlds', 'empty_world.world')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )
    
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    # Delay spawning of robots so Gazebo has time to start and the spawn service is ready
    spawn_model_file = os.path.join(tb3_gz_share, 'models', 'turtlebot3_burger', 'model.sdf')

    spawn_tb3_1 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3_burger_1', '-file', spawn_model_file,
                           '-x', '0.0', '-y', '0.0', '-z', '0.01'],
                output='screen'
            )
        ]
    )

    spawn_tb3_2 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3_burger_2', '-file', spawn_model_file,
                           '-x', '1.0', '-y', '0.0', '-z', '0.01', '-robot_namespace', 'tb3_2'],
                output='screen'
            )
        ]
    )

    spawn_tb3_3 = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3_burger_3', '-file', spawn_model_file,
                           '-x', '0.0', '-y', '1.0', '-z', '0.01', '-robot_namespace', 'tb3_3'],
                output='screen'
            )
        ]
    )

    # UWB Emulator Node (use sim time)
    uwb_emulator_node = Node(
        package='int_sys_fp',
        executable='uwb_emulator',
        name='uwb_sensor_emulator',
        arguments=[LaunchConfiguration('noise_type')],
        parameters=[
            {'uwb_sensor_frequency': LaunchConfiguration('uwb_frequency')},
            {'use_sim_time': True}
        ],
        output='screen',
        emulate_tty=True
    )

    # Distributed controller: one regulator_node instance per robot (robot_id 0/1/2),
    # replacing the old single centralized controller_node that looped over all 3 robots.
    controller_nodes = [
        Node(
            package='int_sys_fp',
            executable='regulator_node',
            name=f'controller_node_{i}',
            parameters=[{'robot_id': i}, {'use_sim_time': True}],
            output='screen',
            emulate_tty=True
        )
        for i in range(3)
    ]

    # Pose filter: one pose_ekf_node or pose_ukf_node instance per robot, each
    # estimating its own [x,y,theta] from UWB (x,y) + IMU (theta) - replaces the
    # old single distance-space KF/UKF node.
    pose_ekf_nodes = [
        Node(
            package='int_sys_fp',
            executable='pose_ekf_node',
            name=f'pose_ekf_node_{i}',
            parameters=[{'robot_id': i}, {'use_sim_time': True}],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ekf'"]))
        )
        for i in range(3)
    ]

    pose_ukf_nodes = [
        Node(
            package='int_sys_fp',
            executable='pose_ukf_node',
            name=f'pose_ukf_node_{i}',
            parameters=[{'robot_id': i}, {'use_sim_time': True}],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ukf'"]))
        )
        for i in range(3)
    ]

    # Legacy distance-space C++ Kalman Filter (kept as baseline for comparison in the
    # report, not consumed by the distributed controllers above)
    legacy_kf_cpp_node = Node(
        package='int_sys_fp',
        executable='distance_kf_node',
        name='distance_kf_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_legacy_kf'))
    )

    # Trajectory Planner Node (optional) - Circular trajectory around origin
    planner_node = Node(
        package='int_sys_fp',
        executable='trajectory_planner.py',
        name='trajectory_planner',
        parameters=[
            {'radius': 5.0},             # Circle radius in meters (outside anchor area)
            {'angular_velocity': 0.05},    # rad/s (slower for large radius)
            {'update_rate': 50.0},        # Hz (same as controller)
            {'use_sim_time': True}
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_planner'))
    )

    ld = LaunchDescription([
        uwb_frequency_arg,
        noise_type_arg,
        enable_planner_arg,
        filter_type_arg,
        enable_legacy_kf_arg,
        enable_bag_record_arg,
        enable_plotjuggler_arg,
        set_tb3_model,
        set_gazebo_plugin_path,
        create_bags_dir,
        gazebo_launch,
        gzclient_launch,
        spawn_tb3_1,
        spawn_tb3_2,
        spawn_tb3_3,
        uwb_emulator_node,
        *controller_nodes,
        *pose_ekf_nodes,
        *pose_ukf_nodes,
        legacy_kf_cpp_node,
        planner_node,
        plotjuggler_node,
        delayed_bag_record
    ])

    return ld