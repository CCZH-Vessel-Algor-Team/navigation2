"""
COLREGS local planner demo — primary development entry for the server stack.

Runs the full colregs server chain (colregs_local_planner_server with the
in-process colregs_ts_state sub-node via the custom colregs bringup) in an
enlarged walled world: boundary walls at +-15 m (30x30 m, slightly larger
than the 24x24 m reference world), three spread-out pillars, the target ship
crossing a centered vertical corridor (y in [-1, 9] at x = 8, axis locked to
world y), and AMCL auto-initialized at the ownship
spawn pose (no manual "2D Pose Estimate" required for the default
configuration).

The legacy planner_server + VORRTStar + standalone TS-node stack remains
available via colregs_ts_projection_validation_launch.py.
"""

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
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    ts_x_pose = LaunchConfiguration('ts_x_pose')
    ts_y_pose = LaunchConfiguration('ts_y_pose')
    ts_yaw = LaunchConfiguration('ts_yaw')
    ts_speed = LaunchConfiguration('ts_speed')
    ts_pingpong_distance = LaunchConfiguration('ts_pingpong_distance')
    ts_motion_axis_mode = LaunchConfiguration('ts_motion_axis_mode')

    colregs_bringup_launch_file = os.path.join(
        bringup_dir, 'launch', 'colregs_bringup_launch.py')

    # The TS state subsystem runs inside colregs_local_planner_server as the
    # colregs_ts_state lifecycle sub-node (CPA markers on /cpa_markers,
    # decision markers on /colregs_decision_markers).
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
            'world': world,
            'map': map_yaml,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'ts_x_pose': ts_x_pose,
            'ts_y_pose': ts_y_pose,
            'ts_yaw': ts_yaw,
            'ts_speed': ts_speed,
            'ts_pingpong_distance': ts_pingpong_distance,
            'ts_motion_axis_mode': ts_motion_axis_mode,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'params',
                'nav2_colregs_params_local_planner_demo.yaml')),
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
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                bringup_dir, 'worlds', 'colregs_open_world.sdf.xacro'),
            description='Enlarged walled world: +-15 m walls, three pillars'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                bringup_dir, 'maps', 'colregs_open_map.yaml'),
            description='36x36 m map matching colregs_open_world'),
        DeclareLaunchArgument(
            'x_pose', default_value='6.00',
            description='Ownship spawn x; must match amcl.initial_pose'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.00',
            description='Ownship spawn y; must match amcl.initial_pose'),
        DeclareLaunchArgument(
            'yaw', default_value='0.00',
            description='Ownship spawn yaw; must match amcl.initial_pose'),
        DeclareLaunchArgument(
            'ts_x_pose', default_value='8.00',
            description='Target ship spawn x (crossing the patrol corridor)'),
        DeclareLaunchArgument(
            'ts_y_pose', default_value='9.00',
            description='Target ship spawn y (north end of the corridor)'),
        DeclareLaunchArgument(
            'ts_yaw', default_value='-1.5708',
            description='Target ship heading (south = toward patrol corridor)'),
        DeclareLaunchArgument(
            'ts_speed', default_value='0.4',
            description='Target ship speed in m/s'),
        DeclareLaunchArgument(
            'ts_pingpong_distance', default_value='10.0',
            description='Target ship ping-pong travel distance in m'),
        DeclareLaunchArgument(
            'ts_motion_axis_mode', default_value='y',
            description=(
                'Lock the ping-pong axis to world y so the corridor '
                'stays vertical regardless of spawn-yaw capture; '
                'auto/x/angle also accepted')),
        simulation,
    ])
