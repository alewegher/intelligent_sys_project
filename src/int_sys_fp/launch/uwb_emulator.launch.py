#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the package directory
    package_dir = get_package_share_directory('int_sys_fp')
    
    # Declare launch arguments
    sensor_params_file_arg = DeclareLaunchArgument(
        'sensor_params_file',
        default_value=os.path.join(package_dir, '..', '..', 'src', 'int_sys_fp', 'sensor_params.yaml'),
        description='Path to the sensor parameters YAML file'
    )
    
    # UWB Emulator Node
    uwb_emulator_node = Node(
        package='int_sys_fp',
        executable='uwb_emulator',
        name='uwb_sensor_emulator',
        output='screen',
        parameters=[
            {'sensor_params_file': LaunchConfiguration('sensor_params_file')}
        ]
    )
    
    return LaunchDescription([
        sensor_params_file_arg,
        uwb_emulator_node
    ])
