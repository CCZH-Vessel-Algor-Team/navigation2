import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_colregs_vector_object_server')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params',
                                   'vector_object_server_params.yaml'))
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time',
                                                 default_value='true')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    vector_object_server = Node(
        package='nav2_colregs_vector_object_server',
        executable='vector_object_server',
        name='vector_object_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    keepout_costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        condition=None,
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout_zone',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {
                'node_names': [
                    'vector_object_server',
                    'keepout_costmap_filter_info_server',
                ]
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(vector_object_server)
    ld.add_action(keepout_costmap_filter_info_server)
    ld.add_action(lifecycle_manager)
    return ld
