"""Opt-in v2 stack. Owns the navigation processes; do not run another navigator.

Localization, odometry, perception and FCU input arbitration remain deployment
inputs. The optional thruster bridge is specifically the Gazebo backend.
"""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('usv_autonomy')
    bt_share = get_package_share_directory('usv_autonomy_bt')

    def setup(context):
        def arg(key):
            return LaunchConfiguration(key).perform(context)
        ns = arg('namespace').strip('/')
        # The CCS wrapper already uses PushRosNamespace. Own the requested
        # vessel namespace absolutely instead of adding it a second time.
        ros_namespace = '/' + ns if ns else '/'
        simulated = arg('use_sim_time').lower() == 'true'
        tests = arg('simulation_test_controls').lower() == 'true'
        simulated_authority = arg('simulation_authority').lower() == 'true'
        stand_on = arg('left_crossing_stand_on').lower() == 'true'
        if tests and arg('simulation_thrusters').lower() != 'true':
            raise ValueError('simulation fault-test controls require the simulation backend')
        if simulated_authority and (not tests or arg('authority_adapter').lower() == 'true'):
            raise ValueError('simulated authority requires test controls and no FCU authority adapter')
        base = ns + '/base_link' if ns else 'base_link'
        common = [arg('autonomy_params'), {'use_sim_time': simulated, 'enable_simulation_tests': tests,
                                         'left_crossing_stand_on': stand_on}]
        situation_topic = ('/' + ns + '/maritime/stand_on_situations' if ns else
                           '/maritime/stand_on_situations') if stand_on else arg('situation_topic')
        nodes = [
            Node(package='usv_autonomy', executable='regular_safety', namespace=ros_namespace,
                 output='screen', parameters=common),
            Node(package='usv_autonomy', executable='mission', namespace=ros_namespace, output='screen',
                 parameters=common + [{'global_frame': 'map', 'base_frame': base,
                    'database': arg('database'),
                     'navigation_tree': bt_share + '/trees/navigate_to_pose.xml',
                     'navigation_through_tree': bt_share + '/trees/navigate_through_poses.xml',
                     'stand_on_navigation_tree': bt_share + '/trees/stand_on_to_pose.xml',
                     'stand_on_navigation_through_tree': bt_share + '/trees/stand_on_through_poses.xml'}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
            Node(package='usv_autonomy', executable='motion_gate', namespace=ros_namespace, output='screen',
                 parameters=common + [{'radar_frame': base}],
                 remappings=[('safety/points', arg('safety_points')),
                             ('safety/radar_sector', arg('safety_sector')),
                             ('/tf', 'tf'), ('/tf_static', 'tf_static')]),
            Node(package='usv_autonomy', executable='maritime_rules', namespace=ros_namespace, output='screen',
                   parameters=common + ([{'rules_checkpoint': arg('rules_checkpoint')}]
                       if arg('rules_checkpoint').strip() else []),
                   remappings=[('maritime/situations', situation_topic)]),
            Node(package='usv_autonomy_bt', executable='task_tree', namespace=ros_namespace, output='screen',
                 parameters=[{'use_sim_time': simulated}]),
        ]
        if stand_on:
            # Use the SAME ground-truth tracks as the TS/VO chain in this
            # explicitly enabled single-target simulation profile.
            nodes.append(Node(package='nav2_maritime_situation_monitor',
                executable='maritime_situation_monitor', name='stand_on_situation_monitor',
                namespace=ros_namespace, output='screen', parameters=[{
                    'use_sim_time': simulated, 'publish_frequency': 5.0,
                    'tracked_ship_topic': arg('stand_on_tracked_ship_topic'),
                    'odom_topic': ('/' + ns + '/odom') if ns else '/odom',
                    'global_frame': 'map', 'base_frame': base, 'enable_markers': False,
                    'output_topic': situation_topic}],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
        if tests:
            nodes.append(Node(package='usv_autonomy', executable='simulation_controls', namespace=ros_namespace,
                               output='screen', parameters=[{'use_sim_time': simulated, 'simulation_mode': True,
                                                           'simulate_authority': simulated_authority}]))
        if arg('authority_adapter').lower() == 'true':
            nodes.append(Node(package='usv_autonomy', executable='authority_adapter', namespace=ros_namespace,
                               output='screen', parameters=common + [{
                                   'fcu_state_topic': arg('fcu_state_topic'),
                                   'fcu_state_clock': arg('fcu_state_clock')}]))
        if arg('launch_nav2').lower() == 'true':
            params_path = arg('nav2_params')
            if not params_path:
                raise ValueError('nav2_params is required when launch_nav2=true')
            with open(params_path, encoding='utf-8') as stream:
                config = yaml.safe_load(stream)
            def replace(value):
                if isinstance(value, dict):
                    return {k: replace(v) for k, v in value.items()}
                if isinstance(value, list):
                    return [replace(v) for v in value]
                return value.replace('__ROBOT_NS__', ns) if isinstance(value, str) else value
            config = replace(config)
            if stand_on:
                planner = config.get('planner_server', {}).get('ros__parameters', {})
                if not {'GridBased', 'VORRTStar'}.issubset(planner.get('planner_plugins', [])):
                    raise ValueError('left_crossing_stand_on requires GridBased and VORRTStar')
                global_map = config.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
                ts_layer = global_map.get('ts_projection_layer', {})
                if ts_layer.get('plugin') == 'nav2_colregs_costmap_layers::TSProjectionLayer':
                    ts_layer['stand_on_policy_topic'] = ('/' + ns if ns else '') + '/autonomy/maritime'
            navigator = config.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
            navigator.update({
                'require_managed_execution': True,
                'default_nav_to_pose_bt_xml': bt_share + '/trees/navigate_to_pose.xml',
                'default_nav_through_poses_bt_xml': bt_share + '/trees/navigate_through_poses.xml',
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_compute_path_through_poses_action_bt_node',
                    'nav2_follow_path_action_bt_node', 'nav2_wait_action_bt_node',
                    'nav2_remove_passed_goals_action_bt_node',
                    'nav2_rate_controller_bt_node', 'nav2_recovery_node_bt_node',
                    'nav2_pipeline_sequence_bt_node', 'usv_navigation_contract_bt_node'],
            })
            config.setdefault('controller_server', {}).setdefault('ros__parameters', {}).update(
                {'require_managed_execution': True})
            behavior = config.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
            behavior['behavior_plugins'] = ['wait']
            behavior['wait'] = {'plugin': 'nav2_behaviors/Wait'}
            # No unneeded waypoint follower, smoother, raw cmd_vel actuator bridge or
            # RViz Action client compiled against an older nav2_msgs wire layout.
            for name in ('planner_server', 'controller_server', 'bt_navigator', 'behavior_server'):
                config.setdefault(name, {}).setdefault('ros__parameters', {})['use_sim_time'] = simulated
            handle = tempfile.NamedTemporaryFile(mode='w', prefix='usv_bt_v2_', suffix='.yaml', delete=False)
            yaml.safe_dump({ns: config} if ns else config, handle)
            handle.close()
            nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=lambda _: os.unlink(handle.name) if os.path.exists(handle.name) else None)])))
            for package, executable in [('nav2_planner', 'planner_server'),
                                         ('nav2_controller', 'controller_server'),
                                         ('nav2_behaviors', 'behavior_server'),
                                         ('nav2_bt_navigator', 'bt_navigator')]:
                nodes.append(Node(package=package, executable=executable, namespace=ros_namespace,
                                  output='screen', parameters=[handle.name],
                                  remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
            nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_bt_v2', namespace=ros_namespace, output='screen',
                parameters=[{'use_sim_time': simulated, 'autostart': True,
                    'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}]))
        if arg('simulation_thrusters').lower() == 'true':
            thruster_parameters = [arg('thruster_params')] if arg('thruster_params') else []
            nodes.append(Node(package='usv_sim_full', executable='cmd_vel_to_thruster', namespace=ros_namespace,
                output='screen', parameters=thruster_parameters + [{'namespace': ns, 'use_sim_time': simulated,
                                              'managed_output': True}]))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='usv_1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autonomy_params', default_value=share + '/config/autonomy.yaml'),
        DeclareLaunchArgument('database', default_value=os.path.expanduser('~/.ros/usv_autonomy/tasks.sqlite3')),
        DeclareLaunchArgument('rules_checkpoint', default_value='',
                              description='Optional explicit checkpoint; empty preserves autonomy_params/default'),
        DeclareLaunchArgument('launch_nav2', default_value='true'),
        DeclareLaunchArgument('nav2_params', default_value=''),
        DeclareLaunchArgument('authority_adapter', default_value='true'),
        DeclareLaunchArgument('fcu_state_topic', default_value='/usv1/cc/state'),
        DeclareLaunchArgument('fcu_state_clock', default_value='system',
                              description='Source stamp clock, independent of BT use_sim_time: system or ros'),
        DeclareLaunchArgument('simulation_thrusters', default_value='false'),
        DeclareLaunchArgument('simulation_test_controls', default_value='false'),
        DeclareLaunchArgument('simulation_authority', default_value='false',
                              description='Synthetic FCU authority; forbidden with the real authority adapter'),
        DeclareLaunchArgument('thruster_params', default_value=''),
        DeclareLaunchArgument('safety_points', default_value='safety/points'),
        DeclareLaunchArgument('safety_sector', default_value='safety/radar_sector'),
        DeclareLaunchArgument('situation_topic', default_value='/maritime_situation'),
        DeclareLaunchArgument('left_crossing_stand_on', default_value='false'),
        DeclareLaunchArgument('stand_on_tracked_ship_topic', default_value='/dynamic_ship/tracked_ships'),
        OpaqueFunction(function=setup),
    ])
