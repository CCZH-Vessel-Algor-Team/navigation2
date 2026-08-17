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
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    log_level = LaunchConfiguration('log_level')
    colregs_bringup_launch_file = os.path.join(
        bringup_dir, 'launch', 'colregs_bringup_launch.py')

    # The TS state subsystem runs inside colregs_local_planner_server as the
    # colregs_ts_state lifecycle sub-node (CPA markers on /cpa_markers).
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'colregs_ts_simulation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'headless': headless,
            'use_composition': use_composition,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file,
            'bringup_launch_file': colregs_bringup_launch_file,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'params',
                'nav2_colregs_params_ts_projection_validation.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='False',
            description='Whether to execute gzclient'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                bringup_dir, 'rviz', 'colregs_local_planner_demo.rviz')),
        DeclareLaunchArgument('log_level', default_value='info'),
        simulation,
    ])
