#!/usr/bin/env python3
"""
Demo launch: Gazebo (empty_world) + 3 TurtleBot3 (distributed controller + pose
filter per robot) + UWB emulator + trajectory planner + PlotJuggler (live view).

No bag recording - use synchronized_system_with_bag.launch.py for that.
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from simulation_common import declare_common_args, build_simulation_actions, validate_config


def generate_launch_description():
    return LaunchDescription([
        *declare_common_args(),
        OpaqueFunction(function=validate_config),
        *build_simulation_actions(),
    ])
