"""TS subsystem launch - TS State Manager + Avoidance Point + COLREGS Barrier."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_colregs_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tracked_ship_topic = LaunchConfiguration('tracked_ship_topic')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    odom_topic = LaunchConfiguration('odom_topic')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_colregs_params_humble_minimal.yaml'))
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_tracked_ship_topic = DeclareLaunchArgument(
        'tracked_ship_topic', default_value='/dynamic_ship/tracked_ships')
    declare_robot_base_frame = DeclareLaunchArgument(
        'robot_base_frame', default_value='base_link')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='odom')

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_tracked_ship_topic,
        declare_robot_base_frame,
        declare_odom_topic,
        Node(
            package='nav2_colregs_ts_manager',
            executable='ts_state_manager',
            name='ts_state_manager',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'tracked_ship_topic': tracked_ship_topic,
                'robot_base_frame': robot_base_frame,
                'odom_topic': odom_topic,
                'tcpa_horizon': 15.0,
                'safety_factor': 1.5,
                'os_radius': 5.0,
            }],
        ),
        Node(
            package='nav2_colregs_ts_manager',
            executable='avoidance_point_node',
            name='avoidance_point_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_colregs_ts_manager',
            executable='barrier_node',
            name='barrier_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
