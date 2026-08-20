#!/usr/bin/env python3
"""
Shared building blocks for the two launch entry points of this package:
  - synchronized_system.launch.py           (demo: sim + PlotJuggler, no bag)
  - synchronized_system_with_bag.launch.py   (sim + PlotJuggler + ros2 bag record)

Not a launch file itself (no generate_launch_description()) - imported by both,
so the Gazebo/spawn/controller/filter setup only exists in one place.
"""

import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def declare_common_args():
    """Launch arguments shared by both entry points."""
    return [
        DeclareLaunchArgument(
            'uwb_frequency', default_value='50.0',
            description='UWB sensor frequency in Hz'),
        DeclareLaunchArgument(
            'noise_type', default_value='1',
            description='Noise type for UWB sensor: 1=Gaussian, 2=Uniform'),
        DeclareLaunchArgument(
            'enable_planner', default_value='true',
            description='Enable trajectory planner node (circular trajectory)'),
        DeclareLaunchArgument(
            'filter_type', default_value='ekf',
            description="Pose filter to use for all 3 robots: 'ekf' or 'ukf'"),
        DeclareLaunchArgument(
            'enable_legacy_kf', default_value='false',
            description='Also launch the legacy distance-space C++ Kalman Filter '
                         '(baseline for comparison, not used by the controllers)'),
        DeclareLaunchArgument(
            'enable_plotjuggler', default_value='true',
            description='Launch PlotJuggler for live topic visualization '
                         '(requires ros-humble-plotjuggler)'),
    ]


def build_simulation_actions():
    """Gazebo + 3 TurtleBot3 spawns + UWB emulator + 3x distributed controller +
    3x pose filter (ekf or ukf, per filter_type) + trajectory planner + PlotJuggler.
    Does NOT include bag recording - added separately by whichever launch file wants it.
    """
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    set_gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', '/opt/ros/humble/lib']
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tb3_gz_share = get_package_share_directory('turtlebot3_gazebo')
    world_file = os.path.join(tb3_gz_share, 'worlds', 'empty_world.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py'))
    )

    spawn_model_file = os.path.join(tb3_gz_share, 'models', 'turtlebot3_burger', 'model.sdf')

    spawn_tb3_1 = TimerAction(period=6.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_burger_1', '-file', spawn_model_file,
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen')])

    spawn_tb3_2 = TimerAction(period=10.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_burger_2', '-file', spawn_model_file,
                   '-x', '1.0', '-y', '0.0', '-z', '0.01', '-robot_namespace', 'tb3_2'],
        output='screen')])

    spawn_tb3_3 = TimerAction(period=13.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_burger_3', '-file', spawn_model_file,
                   '-x', '0.0', '-y', '1.0', '-z', '0.01', '-robot_namespace', 'tb3_3'],
        output='screen')])

    uwb_emulator_node = Node(
        package='int_sys_fp', executable='uwb_emulator', name='uwb_sensor_emulator',
        arguments=[LaunchConfiguration('noise_type')],
        parameters=[
            {'uwb_sensor_frequency': LaunchConfiguration('uwb_frequency')},
            {'use_sim_time': True}
        ],
        output='screen', emulate_tty=True)

    # Distributed controller: one regulator_node instance per robot (robot_id 0/1/2)
    controller_nodes = [
        Node(package='int_sys_fp', executable='regulator_node', name=f'controller_node_{i}',
             parameters=[{'robot_id': i}, {'use_sim_time': True}],
             output='screen', emulate_tty=True)
        for i in range(3)
    ]

    # Pose filter: one pose_ekf_node or pose_ukf_node instance per robot
    pose_ekf_nodes = [
        Node(package='int_sys_fp', executable='pose_ekf_node', name=f'pose_ekf_node_{i}',
             parameters=[{'robot_id': i}, {'use_sim_time': True}],
             output='screen', emulate_tty=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ekf'"])))
        for i in range(3)
    ]
    pose_ukf_nodes = [
        Node(package='int_sys_fp', executable='pose_ukf_node', name=f'pose_ukf_node_{i}',
             parameters=[{'robot_id': i}, {'use_sim_time': True}],
             output='screen', emulate_tty=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ukf'"])))
        for i in range(3)
    ]

    # Legacy distance-space C++ Kalman Filter (baseline for comparison, not on the control path)
    legacy_kf_cpp_node = Node(
        package='int_sys_fp', executable='distance_kf_node', name='distance_kf_node',
        parameters=[{'use_sim_time': True}], output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_legacy_kf')))

    planner_node = Node(
        package='int_sys_fp', executable='trajectory_planner.py', name='trajectory_planner',
        parameters=[
            {'radius': 5.0},
            {'angular_velocity': 0.05},
            {'update_rate': 50.0},
            {'use_sim_time': True}
        ],
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_planner')))

    plotjuggler_node = Node(
        package='plotjuggler', executable='plotjuggler', name='plotjuggler',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('enable_plotjuggler')))

    return [
        set_tb3_model,
        set_gazebo_plugin_path,
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
    ]


BAG_TOPICS = [
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
