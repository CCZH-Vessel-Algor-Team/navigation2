"""TS subsystem launch — TS State Manager + Avoidance Point + COLREGS Barrier."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tracked_ship_topic = LaunchConfiguration('tracked_ship_topic')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    odom_topic = LaunchConfiguration('odom_topic')

    declare_params_file = DeclareLaunchArgument('params_file')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_tracked_ship_topic = DeclareLaunchArgument(
        'tracked_ship_topic', default_value='/tracked_ship')
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
