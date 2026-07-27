from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_colregs_local_planner_server',
            executable='colregs_local_planner_server',
            name='colregs_local_planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_colregs_local_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['colregs_local_planner_server'],
            }],
        ),
    ])
