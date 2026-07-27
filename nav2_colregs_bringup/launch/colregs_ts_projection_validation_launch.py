import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition


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

    local_planner_server = LifecycleNode(
        package='nav2_colregs_local_planner_server',
        executable='colregs_local_planner_server',
        name='colregs_local_planner_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_colregs_local_planner = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_colregs_local_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['colregs_local_planner_server'],
        }],
    )

    start_base_on_local_planner_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=local_planner_server,
            goal_state='active',
            entities=[base_launch],
            handle_once=True,
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_headless)
    ld.add_action(ts_subsystem)
    ld.add_action(local_planner_server)
    ld.add_action(start_base_on_local_planner_active)
    ld.add_action(lifecycle_manager_colregs_local_planner)
    return ld
