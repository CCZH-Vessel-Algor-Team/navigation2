# Copyright (c) 2024 COLREGS Local Planner Project
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

"""COLREGS TS simulation — empty world + TB3 ownship + Target Ship obstacle."""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                             ExecuteProcess, IncludeLaunchDescription,
                             OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_colregs_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # Navigation parameters
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    graph_filepath = LaunchConfiguration('graph')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Simulation parameters
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # Ownship parameters
    ownship_pose = {
        'x': LaunchConfiguration('x_pose', default='6.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Target Ship parameters
    ts_x = LaunchConfiguration('ts_x_pose', default='8.50')
    ts_y = LaunchConfiguration('ts_y_pose', default='4.50')
    ts_yaw = LaunchConfiguration('ts_yaw', default='-1.5708')
    ts_speed = LaunchConfiguration('ts_speed', default='0.3')
    ts_pingpong_distance = LaunchConfiguration('ts_pingpong_distance',
                                               default='6.0')
    enable_ts_motion = LaunchConfiguration('enable_ts_motion')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # --- Declare arguments ---
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps',
                                   'colregs_reference_map.yaml'),
        description='Full path to map yaml file to load')
    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(nav2_bringup_dir, 'graphs',
                                   'turtlebot3_graph.geojson'))
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params',
                                   'nav2_colregs_params.yaml'))
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz',
                                   'nav2_default_view.rviz'))
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub', default_value='True')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True')
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Whether to execute gzclient')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds',
                                   'colregs_reference_world.sdf.xacro'),
        description='Full path to world model file to load')
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot3_waffle')
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'))

    # Target Ship arguments
    declare_ts_x_cmd = DeclareLaunchArgument(
        'ts_x_pose', default_value='8.50',
        description='TS initial X position (ahead of ownship)')
    declare_ts_y_cmd = DeclareLaunchArgument(
        'ts_y_pose', default_value='4.50',
        description='TS initial Y position (left-front side)')
    declare_ts_yaw_cmd = DeclareLaunchArgument(
        'ts_yaw', default_value='-1.5708',
        description='TS initial yaw for crossing trajectory')
    declare_ts_speed_cmd = DeclareLaunchArgument(
        'ts_speed', default_value='0.3',
        description='TS forward speed (m/s)')
    declare_ts_pingpong_distance_cmd = DeclareLaunchArgument(
        'ts_pingpong_distance', default_value='6.0',
        description='TS one-way distance before reverse (m)')
    declare_enable_ts_motion_cmd = DeclareLaunchArgument(
        'enable_ts_motion', default_value='True',
        description='Enable ping-pong motion command publisher for target ship')

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # --- Ownship ---
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        remappings=remappings,
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': ownship_pose['x'],
            'y_pose': ownship_pose['y'],
            'z_pose': ownship_pose['z'],
            'roll': ownship_pose['R'],
            'pitch': ownship_pose['P'],
            'yaw': ownship_pose['Y'],
        }.items(),
    )

    # --- Target Ship (custom spawn + minimal bridge) ---
    target_ship_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='target_ship',
        parameters=[{
            'config_file': os.path.join(
                bringup_dir, 'configs', 'target_ship_bridge.yaml'),
            'expand_gz_topic_names': True,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    target_ship_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace='target_ship',
        arguments=[
            '-name', 'target_ship',
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=target_ship ',
                os.path.join(bringup_dir, 'models',
                             'target_ship_simple.sdf.xacro')]),
            '-x', ts_x,
            '-y', ts_y,
            '-z', '0.12',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', ts_yaw,
        ],
    )

    target_ship_state_publisher = Node(
        executable=os.path.join(bringup_dir, 'scripts',
                                'target_ship_state_publisher.py'),
        name='target_ship_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gz_pose_topic': '/world/default/pose/info',
            'target_model_name': 'target_ship',
            'tracked_frame_id': 'map',
            'tf_child_frame_id': 'ts_virtual_base_link',
        }],
    )

    target_ship_motion_commander = Node(
        executable=os.path.join(bringup_dir, 'scripts',
                                'target_ship_motion_commander.py'),
        name='target_ship_motion_commander',
        output='screen',
        condition=IfCondition(enable_ts_motion),
        parameters=[{
            'use_sim_time': use_sim_time,
            'speed': ts_speed,
            'pingpong_distance': ts_pingpong_distance,
        }],
    )

    # --- Gazebo ---
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_simulator),
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # --- GZ_SIM_RESOURCE_PATH ---
    set_env_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(bringup_dir, 'models'))
    set_env_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(sim_dir, 'models'))
    set_env_tb3_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(sim_dir).parent.resolve()))
    set_env_models_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(bringup_dir)).parent.resolve()))

    # --- Build launch ---
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_ts_x_cmd)
    ld.add_action(declare_ts_y_cmd)
    ld.add_action(declare_ts_yaw_cmd)
    ld.add_action(declare_ts_speed_cmd)
    ld.add_action(declare_ts_pingpong_distance_cmd)
    ld.add_action(declare_enable_ts_motion_cmd)

    ld.add_action(set_env_models)
    ld.add_action(set_env_tb3_models)
    ld.add_action(set_env_tb3_parent)
    ld.add_action(set_env_models_parent)

    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(gz_robot)
    ld.add_action(target_ship_bridge)
    ld.add_action(target_ship_spawn)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(target_ship_state_publisher)
    ld.add_action(target_ship_motion_commander)

    return ld
