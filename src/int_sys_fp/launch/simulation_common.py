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
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
                            SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler,
                            EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def declare_common_args(default_sim_duration='0.0'):
    """Launch arguments shared by both entry points.

    default_sim_duration lets the bag entry point default to a bounded run while the demo
    entry point keeps running until Ctrl-C.
    """
    return [
        DeclareLaunchArgument(
            'sim_duration', default_value=default_sim_duration,
            description='Length of the run in SECONDS OF SIMULATION TIME, counted from the '
                        'moment recording starts. 0 = run until Ctrl-C. Measured on /clock '
                        'rather than the wall clock, so runs stay comparable even if '
                        'Gazebo\'s real-time factor varies. One trajectory lap takes '
                        '2*pi/angular_velocity seconds (125.7 s at the default 0.05 rad/s).'),
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
            'filter_type', default_value='ekf', choices=['ekf', 'ukf'],
            description="Pose filter to use for all 3 robots: 'ekf' or 'ukf'"),
        DeclareLaunchArgument(
            'enable_legacy_kf', default_value='false',
            description='Also launch the legacy distance-space C++ Kalman Filter '
                         '(baseline for comparison, not used by the controllers)'),
        DeclareLaunchArgument(
            'enable_plotjuggler', default_value='true',
            description='Launch PlotJuggler for live topic visualization '
                         '(requires ros-humble-plotjuggler)'),
        DeclareLaunchArgument(
            'radius', default_value='3.0',
            description='Radius of the circular reference trajectory, metres. '
                        'Feasibility: radius * angular_velocity must stay below the '
                        'TurtleBot3 burger limit of 0.22 m/s, with margin for the '
                        'formation-keeping motion superimposed on tracking.'),
        DeclareLaunchArgument(
            'angular_velocity', default_value='0.05',
            description='Angular velocity of the reference trajectory, rad/s.'),
        DeclareLaunchArgument(
            'uwb_source', default_value='raw', choices=['raw', 'filtered'],
            description="UWB topics the pose filters subscribe to. 'filtered' routes them "
                        'through the legacy distance KF + MAD detector and REQUIRES '
                        'enable_legacy_kf:=true.'),
        DeclareLaunchArgument(
            'enable_mad', default_value='true', choices=['true', 'false'],
            description='Enable the MAD outlier detector inside the legacy distance KF.'),
        DeclareLaunchArgument(
            'mad_window', default_value='5',
            description='MAD sliding-window size, samples.'),
        DeclareLaunchArgument(
            'mad_kappa', default_value='3.0',
            description='MAD threshold multiplier: outlier if |z - median| > kappa*scale*MAD.'),
        DeclareLaunchArgument(
            'mad_scale', default_value='1.0',
            description='Gaussian-consistency factor for MAD. 1.0 = historical behaviour '
                        '(kappa=3 is then ~2.02 sigma); use 1.4826 for a true 3-sigma rule.'),
        DeclareLaunchArgument(
            'mad_cov_inflation', default_value='1.5',
            description='Covariance inflation factor applied when an outlier is corrected.'),
        DeclareLaunchArgument(
            'gain_mode', default_value='one_step_riccati',
            choices=['one_step_riccati', 'sdre_ci_experimental'],
            description='Estimator gain strategy. sdre_ci_experimental additionally solves '
                        'the DARE at the frozen operating point and fuses that estimate '
                        'with the classic one via Covariance Intersection.'),
        DeclareLaunchArgument(
            'ci_weight', default_value='0.5',
            description='Covariance Intersection weight w in [0,1]. 0.5 = plain average of '
                        'the two estimates; 1.0 reproduces the classic filter exactly.'),
        DeclareLaunchArgument(
            'ci_weight_mode', default_value='fixed', choices=['fixed', 'min_trace'],
            description='fixed = use ci_weight; min_trace = pick w minimising trace(P_ci). '
                        'NOTE min_trace degenerates to w=1 unless paired with '
                        'sdre_cov_mode:=steady_state.'),
        DeclareLaunchArgument(
            'sdre_cov_mode', default_value='propagated',
            choices=['propagated', 'steady_state'],
            description='Covariance attributed to the SDRE estimate: propagated (its actual '
                        'covariance given the current prior) or steady_state (the DARE '
                        'design covariance).'),
        DeclareLaunchArgument(
            'ci_feedback', default_value='true', choices=['true', 'false'],
            description='true = the fused estimate drives the control loop. false = shadow '
                        'mode: branches are published but the classic estimate stays on the '
                        'control path, so all estimators see an identical trajectory.'),
        DeclareLaunchArgument(
            'distance_process_noise', default_value='0.01',
            description='Process noise of the legacy scalar distance KF. At the default, '
                        'its steady-state gain is ~0.99 (near pass-through), so '
                        'uwb_source:=filtered looks like raw unless MAD fires. Lower it to '
                        'make the distance-filtering axis visible.'),
    ]


def validate_config(context, *args, **kwargs):
    """Fail at launch time on configurations that would otherwise hang silently.

    uwb_source:=filtered subscribes to topics that only distance_kf_node publishes; if
    that node is not launched, the pose filters block forever in their data guard with no
    error. Deliberately NOT auto-enabling the legacy node: silently starting something the
    user did not ask for makes the recorded configuration ambiguous, which is exactly what
    wrecks experiment bookkeeping.
    """
    uwb_source = LaunchConfiguration('uwb_source').perform(context)
    legacy = LaunchConfiguration('enable_legacy_kf').perform(context).lower()
    if uwb_source == 'filtered' and legacy not in ('true', '1'):
        raise RuntimeError(
            'uwb_source:=filtered requires the legacy distance KF to be running, but '
            'enable_legacy_kf:={}. Re-run with enable_legacy_kf:=true.'.format(legacy))
    return []


def _filter_common_params():
    """Parameters shared by pose_ekf_node and pose_ukf_node.

    Both filters must receive the identical set, otherwise switching filter_type would
    silently change more than the estimator. ParameterValue with an explicit value_type
    is required: launch_ros runs yaml.safe_load() on the substituted string, so a bare
    LaunchConfiguration for e.g. 'ci_weight:=1' yields an int and the node's
    declare_parameter<double> throws InvalidParameterTypeException at startup.
    """
    return {
        'noise_type': ParameterValue(LaunchConfiguration('noise_type'), value_type=int),
        'uwb_source': ParameterValue(LaunchConfiguration('uwb_source'), value_type=str),
        'gain_mode': ParameterValue(LaunchConfiguration('gain_mode'), value_type=str),
        'ci_weight': ParameterValue(LaunchConfiguration('ci_weight'), value_type=float),
        'ci_weight_mode': ParameterValue(LaunchConfiguration('ci_weight_mode'), value_type=str),
        'sdre_cov_mode': ParameterValue(LaunchConfiguration('sdre_cov_mode'), value_type=str),
        'ci_feedback': ParameterValue(LaunchConfiguration('ci_feedback'), value_type=bool),
    }


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
             parameters=[{'robot_id': i}, {'use_sim_time': True}, _filter_common_params()],
             output='screen', emulate_tty=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ekf'"])))
        for i in range(3)
    ]
    pose_ukf_nodes = [
        Node(package='int_sys_fp', executable='pose_ukf_node', name=f'pose_ukf_node_{i}',
             parameters=[{'robot_id': i}, {'use_sim_time': True}, _filter_common_params()],
             output='screen', emulate_tty=True,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('filter_type'), "' == 'ukf'"])))
        for i in range(3)
    ]

    # Legacy distance-space C++ Kalman Filter (baseline for comparison, not on the control path)
    legacy_kf_cpp_node = Node(
        package='int_sys_fp', executable='distance_kf_node', name='distance_kf_node',
        parameters=[{
            'use_sim_time': True,
            'mad_enabled': ParameterValue(LaunchConfiguration('enable_mad'), value_type=bool),
            'mad_window': ParameterValue(LaunchConfiguration('mad_window'), value_type=int),
            'mad_kappa': ParameterValue(LaunchConfiguration('mad_kappa'), value_type=float),
            'mad_scale': ParameterValue(LaunchConfiguration('mad_scale'), value_type=float),
            'mad_cov_inflation': ParameterValue(LaunchConfiguration('mad_cov_inflation'),
                                                value_type=float),
            'distance_process_noise': ParameterValue(
                LaunchConfiguration('distance_process_noise'), value_type=float),
        }], output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_legacy_kf')))

    planner_node = Node(
        package='int_sys_fp', executable='trajectory_planner.py', name='trajectory_planner',
        parameters=[
            {'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float)},
            {'angular_velocity': ParameterValue(LaunchConfiguration('angular_velocity'),
                                                value_type=float)},
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
        #plotjuggler_node,
        *_run_timer_actions(),
    ]


# Recording (and the run-length budget) start after the last robot spawn at t=13 s.
RECORD_START_DELAY = 16.0


def _run_timer_actions():
    """Bounded-run support: stop everything after sim_duration seconds of SIM time.

    Started on the same delay as the bag recorder, so the budget measures RECORDED time
    rather than including the ~16 s of Gazebo startup and robot spawning.

    When run_timer exits, its exit event is turned into a launch Shutdown, which SIGINTs
    every process including `ros2 bag record` - that is what lets rosbag2 close and index
    the bag properly instead of leaving it truncated.
    """
    run_timer_node = Node(
        package='int_sys_fp', executable='run_timer.py', name='run_timer',
        parameters=[{
            'use_sim_time': True,
            'duration': ParameterValue(LaunchConfiguration('sim_duration'), value_type=float),
        }],
        output='screen', emulate_tty=True,
        # PythonExpression rather than IfCondition on the raw value: '0.0' is a truthy
        # non-empty string, so a plain IfCondition would arm the timer even when disabled.
        condition=IfCondition(PythonExpression([LaunchConfiguration('sim_duration'), ' > 0'])))

    return [
        TimerAction(period=RECORD_START_DELAY, actions=[run_timer_node]),
        RegisterEventHandler(OnProcessExit(
            target_action=run_timer_node,
            on_exit=[EmitEvent(event=Shutdown(reason='sim_duration reached'))])),
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
    # sdre_ci_experimental only - absent from one_step_riccati bags, which ros2 bag record
    # tolerates (an explicitly listed topic that never appears is simply not recorded).
    '/pose_debug_riccati', '/tb3_2/pose_debug_riccati', '/tb3_3/pose_debug_riccati',
    '/pose_debug_sdre', '/tb3_2/pose_debug_sdre', '/tb3_3/pose_debug_sdre',
    '/pose_debug_ci', '/tb3_2/pose_debug_ci', '/tb3_3/pose_debug_ci',
    '/gain_mode_debug', '/tb3_2/gain_mode_debug', '/tb3_3/gain_mode_debug',
    '/fsm_state', '/tb3_2/fsm_state', '/tb3_3/fsm_state',
    '/tracking_error', '/tb3_2/tracking_error', '/tb3_3/tracking_error',
    '/centroid_position', '/tb3_2/centroid_position', '/tb3_3/centroid_position',
    '/desired_trajectory', '/desired_trajectory_array',
    '/tb3_2/desired_trajectory_array', '/tb3_3/desired_trajectory_array',
]
