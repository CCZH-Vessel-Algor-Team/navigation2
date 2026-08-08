#!/usr/bin/env python3

# Copyright 2026 vectorwang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the standalone maritime situation monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the standalone monitor launch description."""
    package_dir = get_package_share_directory(
        'nav2_maritime_situation_monitor')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_dir, 'config', 'maritime_situation_monitor.yaml'),
            description='Full path to the monitor parameters file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        Node(
            package='nav2_maritime_situation_monitor',
            executable='maritime_situation_monitor',
            name='maritime_situation_monitor',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
