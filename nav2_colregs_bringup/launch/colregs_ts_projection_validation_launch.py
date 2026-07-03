import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_colregs_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_colregs_params_ts_projection_validation.yaml'))
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

    ts_subsystem = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'ts_subsystem_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_headless)
    ld.add_action(base_launch)
    ld.add_action(ts_subsystem)
    return ld
