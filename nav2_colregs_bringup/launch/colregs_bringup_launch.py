# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    colregs_bringup_dir = get_package_share_directory('nav2_colregs_bringup')
    nav2_launch_dir = os.path.join(bringup_dir, 'launch')
    colregs_launch_dir = os.path.join(colregs_bringup_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    effective_namespace = IfElseSubstitution(
        use_namespace, if_value=namespace, else_value='')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', effective_namespace)},
        condition=IfCondition(use_namespace),
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=effective_namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_group = GroupAction([
        PushROSNamespace(
            condition=IfCondition(use_namespace),
            namespace=effective_namespace),
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'localization_launch.py')),
            launch_arguments={
                'namespace': effective_namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    colregs_launch_dir, 'colregs_navigation_launch.py')),
            launch_arguments={
                'namespace': effective_namespace,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
                'log_level': log_level,
            }.items(),
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_namespace', default_value='false',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Accepted for compatibility; localization is always used'),
        DeclareLaunchArgument(
            'map', default_value='', description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the navigation stack'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composed bringup'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Respawn nodes when composition is disabled'),
        DeclareLaunchArgument('log_level', default_value='info'),
        bringup_group,
    ])
