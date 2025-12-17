#!/usr/bin/env python3

import os
import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file per sistema sincronizzato: Gazebo (empty_world) + 3 TurtleBot3 + UWB Emulator + Controller + KF
    + Registrazione automatica ROS2 Bag (120s)
    """

    # --- CONFIGURAZIONE BAG RECORDING ---
    pkg_name = 'int_sys_fp'
    
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_name = f'int_sys_sim_bag_{timestamp}'

    pkg_share = get_package_share_directory(pkg_name)
    bag_output_dir_base = os.path.join(pkg_share, 'bags')
    bag_output_dir = os.path.join(bag_output_dir_base, bag_name)
    
    topics_to_record = [
        '/centroid_position',
        '/cmd_vel',
        '/tb3_2/cmd_vel',
        '/tb3_3/cmd_vel',
        '/tb3_2/odom',
        '/tb3_3/odom',
        '/odom',
        '/tracking_error',
        '/uwb/anchor_distances',
        '/uwb/filtered_anchor_distances',
        '/uwb/robot_distances',
        '/desired_trajectory',
        '/kf_filtered_robot_pose'
    ]
    
    # Crea la directory 'bags' se non esiste (necessario per ros2 bag record)
    create_bags_dir = ExecuteProcess(
        cmd=['mkdir', '-p', bag_output_dir_base],
        output='screen',
        shell=False
    )

    # Processo di registrazione (con durata limitata a 120s)
    # FIX V3: Torniamo al formato lista, ma assicuriamo il comando sia corretto.
    # L'errore potrebbe essere causato dal fatto che `ros2` non è nel PATH del processo.
    bag_record_cmd = [
        'ros2', 'bag', 'record', 
        '--use-sim-time', 
        '-o', bag_output_dir#, 
        #'--duration', '120s'
    ] + topics_to_record
    
    bag_record_process = ExecuteProcess(
        cmd=bag_record_cmd,
        output='screen',
        # Rimuoviamo shell=True in questo tentativo, poiché a volte interferisce con il PATH
        # Inseriamo un ritardo minimo per la registrazione, nel caso in cui i topic non siano subito disponibili
        # Lo avvieremo dopo la creazione della cartella
    )
    # -------------------------------------

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
    
    use_cpp_kf_arg = DeclareLaunchArgument(
        'use_cpp_kf',
        default_value='true',
        description='Use C++ Kalman Filter implementation instead of Python (for better performance)'
    )

    # Ensure TurtleBot3 model env var is set for gazebo/model resolution
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Include Gazebo without any robot pre-spawned
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
    kf_python_node = Node(
        package='int_sys_fp',
        executable='KF.py',  # script installed into lib/int_sys_fp/KF.py
        name='distance_kf_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(NotSubstitution(LaunchConfiguration('use_cpp_kf')))
    )
    
    # Kalman Filter C++ node (compiled executable for better performance)
    kf_cpp_node = Node(
        package='int_sys_fp',
        executable='distance_kf_node',  # compiled C++ executable
        name='distance_kf_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_cpp_kf'))
    )
    
    # Trajectory Planner Node (optional) - Circular trajectory around origin
    planner_node = Node(
        package='int_sys_fp',
        executable='trajectory_planner.py',
        name='trajectory_planner',
        parameters=[
            {'radius': 5.0},              # Circle radius in meters (outside anchor area)
            {'angular_velocity': 0.05},    # rad/s (slower for large radius)
            {'update_rate': 50.0},         # Hz (same as controller)
            {'use_sim_time': True}
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_planner'))
    )
    
    # Ritardiamo l'avvio della registrazione di 5 secondi per garantire che tutti i nodi siano attivi
    delayed_bag_record_process = TimerAction(
        period=5.0,
        actions=[bag_record_process]
    )

    ld = LaunchDescription([
        uwb_frequency_arg,
        noise_type_arg,
        enable_planner_arg,
        use_cpp_kf_arg,
        set_tb3_model,
        
        # 1. Crea la directory
        create_bags_dir,
        # 2. Avvia il resto della simulazione
        gazebo_launch,
        gzclient_launch,
        spawn_tb3_1,
        spawn_tb3_2,
        spawn_tb3_3,
        uwb_emulator_node,
        controller_node,
        kf_python_node,
        kf_cpp_node,
        planner_node,
        
        # 4. Avvia la registrazione con un ritardo
        delayed_bag_record_process
    ])

    return ld