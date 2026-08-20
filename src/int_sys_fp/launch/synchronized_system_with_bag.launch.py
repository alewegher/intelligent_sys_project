#!/usr/bin/env python3
"""
Same system as synchronized_system.launch.py (Gazebo + 3-robot distributed
controller/filter + UWB emulator + trajectory planner + PlotJuggler), plus a
ros2 bag recording of all estimation/control topics for offline analysis
(e.g. Matlab post-processing of /pose_debug).

Bag is written under ~/ros2_ws/bags/ (workspace root, not install/) so it
survives a `rm -rf install/` clean rebuild.
"""

import os
import sys
import datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from simulation_common import (declare_common_args, build_simulation_actions,
                               validate_config, BAG_TOPICS)


def generate_launch_description():
    enable_bag_record_arg = DeclareLaunchArgument(
        'enable_bag_record', default_value='true',
        description='Record a ros2 bag of all estimation/control topics')

    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_output_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'bags', f'int_sys_sim_{timestamp}')

    create_bags_dir = ExecuteProcess(
        cmd=['mkdir', '-p', os.path.dirname(bag_output_dir)],
        output='screen', shell=False)

    bag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_output_dir] + BAG_TOPICS,
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bag_record')))

    # Start recording once robots are spawned and estimators/controllers are up
    # (last spawn fires at 13s) rather than from t=0.
    delayed_bag_record = TimerAction(period=16.0, actions=[bag_record_process])

    return LaunchDescription([
        *declare_common_args(),
        OpaqueFunction(function=validate_config),
        enable_bag_record_arg,
        create_bags_dir,
        *build_simulation_actions(),
        delayed_bag_record,
    ])
