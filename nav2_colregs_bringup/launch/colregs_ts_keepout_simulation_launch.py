import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_colregs_bringup')

    params_file = LaunchConfiguration('params_file')
    keepout_mask = LaunchConfiguration('keepout_mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_colregs_params_with_keepout.yaml'))
    declare_keepout_mask = DeclareLaunchArgument(
        'keepout_mask',
        default_value=os.path.join(
            bringup_dir, 'maps', 'colregs_reference_keepout_map.yaml'))
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'colregs_ts_simulation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'headless': headless,
        }.items(),
    )

    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': keepout_mask},
            {'use_sim_time': use_sim_time},
        ],
    )

    keepout_costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager_keepout_zone = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout_zone',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {
                'node_names': [
                    'keepout_filter_mask_server',
                    'keepout_costmap_filter_info_server',
                ]
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(declare_keepout_mask)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_headless)
    ld.add_action(base_launch)
    ld.add_action(keepout_filter_mask_server)
    ld.add_action(keepout_costmap_filter_info_server)
    ld.add_action(lifecycle_manager_keepout_zone)
    return ld
