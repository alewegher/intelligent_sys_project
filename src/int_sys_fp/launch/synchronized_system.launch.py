#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
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
        default_value='false',
        description='Enable trajectory planner node'
    )
    
    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='circle',
        description='Trajectory type: circle, line, square, static'
    )

    # Ensure TurtleBot3 model env var is set for gazebo/model resolution
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

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

    # Controller Node
    controller_node = Node(
        package='int_sys_fp',
        executable='regulator_node',
        name='controller_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True
    )

    # Kalman Filter Python node (installed via CMake install(PROGRAMS ...))
    kf_node = Node(
        package='int_sys_fp',
        executable='KF.py',  # script installed into lib/int_sys_fp/KF.py
        name='distance_kf_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True
    )
    
    # Trajectory Planner Node (optional)
    planner_node = Node(
        package='int_sys_fp',
        executable='trajectory_planner.py',
        name='trajectory_planner',
        parameters=[
            {'trajectory_type': LaunchConfiguration('trajectory_type')},
            {'radius': 3.0},
            {'speed': 0.2},
            {'update_rate': 10.0}
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_planner'))
    )

    ld = LaunchDescription([
        uwb_frequency_arg,
        noise_type_arg,
        enable_planner_arg,
        trajectory_type_arg,
        set_tb3_model,
        gazebo_launch,
        gzclient_launch,
        spawn_tb3_1,
        spawn_tb3_2,
        spawn_tb3_3,
        uwb_emulator_node,
        controller_node,
        kf_node,
        planner_node
    ])

    return ld