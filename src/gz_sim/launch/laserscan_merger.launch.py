import os
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    #laserscan_mergerの起動オプション設定
    laserscan_merger_params_file = LaunchConfiguration('laserscan_merger_params_file')
    declare_laserscan_merger_params_cmd = DeclareLaunchArgument(
        'laserscan_merger_params_file',
        default_value=os.path.join(get_package_share_directory("gz_sim"),
                                   'params', 'laserscan_merge.yaml'),
        description='Path to param config in yaml format')
    # laserscan_mergerの起動設定
    laserscan_multi_merger = Node(
        parameters=[laserscan_merger_params_file],
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name="laserscan_multi_merger",
        output='both')


    return LaunchDescription([
        declare_laserscan_merger_params_cmd,
        laserscan_multi_merger,
    ])

