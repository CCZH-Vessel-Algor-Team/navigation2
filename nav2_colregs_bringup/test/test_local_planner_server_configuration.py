import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]
PARAMS_FILE = (
    PACKAGE_DIR / 'params' / 'nav2_colregs_params_ts_projection_validation.yaml'
)
DEFAULT_RVIZ_FILE = PACKAGE_DIR / 'rviz' / 'nav2_default_view.rviz'
DEMO_RVIZ_FILE = PACKAGE_DIR / 'rviz' / 'colregs_local_planner_demo.rviz'


def load_params():
    return yaml.safe_load(PARAMS_FILE.read_text())


def topic_value(display, topic_name='Topic'):
    return display[topic_name]['Value']


def test_default_rviz_profile_is_preserved_byte_for_byte():
    assert hashlib.sha256(DEFAULT_RVIZ_FILE.read_bytes()).hexdigest() == (
        '17a331f5878bde3a5f0c7f13692871f79552ec3cd053f670c1efccb9673ddcfc'
    )


def test_demo_rviz_profile_is_the_dedicated_colregs_derivative():
    default_config = yaml.safe_load(DEFAULT_RVIZ_FILE.read_text())
    demo_text = DEMO_RVIZ_FILE.read_text()
    demo_config = yaml.safe_load(demo_text)

    default_displays = default_config['Visualization Manager']['Displays']
    planner = next(
        display for display in default_displays
        if display.get('Name') == 'Global Planner'
    )
    planner['Name'] = 'COLREGS Planner'
    costmap = next(
        display for display in planner['Displays']
        if display.get('Name') == 'Global Costmap'
    )
    costmap['Name'] = 'COLREGS Costmap'
    costmap['Topic']['Value'] = 'colregs_costmap/costmap'
    costmap['Update Topic']['Value'] = 'colregs_costmap/costmap_updates'
    footprint = next(
        display for display in planner['Displays']
        if topic_value(display) == 'global_costmap/published_footprint'
    )
    footprint['Topic']['Value'] = 'colregs_costmap/published_footprint'
    planner['Displays'] = [
        display for display in planner['Displays']
        if topic_value(display) != 'global_costmap/voxel_layer'
    ]

    assert demo_config == default_config
    assert 'global_costmap/' not in demo_text

    demo_displays = demo_config['Visualization Manager']['Displays']
    assert not any(
        display.get('Name') == 'Global Planner' for display in demo_displays
    )
    planner = next(
        display for display in demo_displays
        if display.get('Name') == 'COLREGS Planner'
    )
    assert not any(
        display.get('Class') == 'rviz_default_plugins/PointCloud2'
        for display in planner['Displays']
    )
    assert topic_value(next(
        display for display in planner['Displays']
        if display.get('Name') == 'Path'
    )) == 'plan'

    panels = demo_config['Panels']
    assert any(
        panel.get('Class') == 'nav2_rviz_plugins/Navigation 2'
        and panel.get('Name') == 'Navigation 2'
        for panel in panels
    )


def test_demo_tree_computes_and_follows_standard_path():
    tree = ET.parse(
        PACKAGE_DIR
        / 'behavior_trees'
        / 'navigate_w_colregs_local_planner_server.xml'
    )

    rate_controllers = tree.findall('.//RateController')
    assert len(rate_controllers) == 1
    assert rate_controllers[0].attrib['hz'] == '1.0'

    compute_path = rate_controllers[0].find('.//ComputePathToPose')
    assert compute_path is not None
    assert compute_path.attrib == {
        'goal': '{goal}',
        'path': '{path}',
        'planner_id': 'RRTStar',
        'error_code_id': '{compute_path_error_code}',
    }
    assert tree.find('.//FollowPath').attrib['path'] == '{path}'

    assert tree.find('.//ControllerSelector') is not None
    assert tree.find('.//GoalUpdated') is not None
    for recovery in ('Spin', 'Wait', 'BackUp'):
        assert tree.find(f'.//{recovery}') is not None

    clear_services = {
        node.attrib['service_name'] for node in tree.findall('.//ClearEntireCostmap')
    }
    assert clear_services == {
        'colregs_costmap/clear_entirely_colregs_costmap',
        'local_costmap/clear_entirely_local_costmap',
    }

    for removed_node in ('PlannerSelector', 'ComputeLocalPath', 'IsPathValid'):
        assert tree.find(f'.//{removed_node}') is None
    assert '{local_path}' not in ET.tostring(tree.getroot(), encoding='unicode')
    assert 'global_costmap' not in ET.tostring(tree.getroot(), encoding='unicode')


def test_projection_validation_parameters_route_to_rrt_star_server():
    params = load_params()
    bt_params = params['bt_navigator']['ros__parameters']

    assert bt_params['navigators'] == ['navigate_to_pose']
    assert 'navigate_through_poses' not in bt_params
    assert 'default_nav_through_poses_bt_xml' not in bt_params
    assert bt_params['default_nav_to_pose_bt_xml'].endswith(
        '/behavior_trees/navigate_w_colregs_local_planner_server.xml'
    )
    assert bt_params['plugin_lib_names'] == [
        'nav2_create_local_path_action_bt_node'
    ]
    assert 'compute_local_path' not in bt_params['error_code_name_prefixes']

    assert 'planner_server' not in params
    assert params['colregs_local_planner_server']['ros__parameters'] == {
        'use_sim_time': True,
        'action_server_result_timeout': 10.0,
        'costmap_update_timeout': 1.0,
        'max_planning_time': 0.8,
        'step_size': 1.0,
        'max_iterations': 1000,
        'goal_bias': 0.1,
        'goal_threshold': 0.5,
        'safety_dist': 0.3,
        'cost_weight': 1.0,
        'max_optimize_iters': 200,
        'eta': 1.1,
        'random_seed': 42,
        'prune_path': True,
    }

    follow_path_params = params['controller_server']['ros__parameters']['FollowPath']
    assert follow_path_params['reset_beta_on_new_goal'] is True
    assert follow_path_params['beta_reset_goal_dist_tolerance'] == 0.05
    assert 'reset_beta_on_new_path' not in follow_path_params

    behavior_params = params['behavior_server']['ros__parameters']
    assert behavior_params['global_costmap_topic'] == (
        'colregs_costmap/costmap_raw'
    )
    assert behavior_params['global_footprint_topic'] == (
        'colregs_costmap/published_footprint'
    )
    assert 'create_local_path' in behavior_params['behavior_plugins']
    assert behavior_params['create_local_path']['plugin'] == (
        'nav2_colregs_local_path_behavior::CreateLocalPath'
    )


def test_colregs_costmap_preserves_the_four_layer_global_map_configuration():
    params = load_params()

    assert 'global_costmap' not in params
    assert params['colregs_costmap']['colregs_costmap']['ros__parameters'] == {
        'update_frequency': 1.0,
        'publish_frequency': 1.0,
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        'rolling_window': False,
        'robot_radius': 0.22,
        'resolution': 0.05,
        'track_unknown_space': True,
        'plugins': [
            'static_layer',
            'obstacle_layer',
            'ts_projection_layer',
            'inflation_layer',
        ],
        'ts_projection_layer': {
            'plugin': 'nav2_colregs_costmap_layers::TSProjectionLayer',
            'enabled': True,
            'tracked_ship_topic': '/tracked_ship',
            'track_timeout': 3.0,
        },
        'obstacle_layer': {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 3.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 2.5,
                'obstacle_min_range': 0.0,
            },
        },
        'static_layer': {
            'plugin': 'nav2_costmap_2d::StaticLayer',
            'map_subscribe_transient_local': True,
        },
        'inflation_layer': {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'cost_scaling_factor': 3.0,
            'inflation_radius': 0.7,
        },
        'always_send_full_costmap': True,
        'introspection_mode': 'disabled',
    }


def test_controller_and_local_costmap_configuration_is_unchanged():
    params = load_params()

    assert params['controller_server']['ros__parameters'] == {
        'controller_frequency': 20.0,
        'costmap_update_timeout': 0.30,
        'min_x_velocity_threshold': 0.001,
        'min_y_velocity_threshold': 0.5,
        'min_theta_velocity_threshold': 0.001,
        'failure_tolerance': 0.3,
        'progress_checker_plugins': ['progress_checker'],
        'goal_checker_plugins': ['general_goal_checker'],
        'controller_plugins': ['FollowPath'],
        'use_realtime_priority': False,
        'speed_limit_topic': 'speed_limit',
        'progress_checker': {
            'plugin': 'nav2_controller::SimpleProgressChecker',
            'required_movement_radius': 0.5,
            'movement_time_allowance': 10.0,
        },
        'general_goal_checker': {
            'plugin': 'nav2_controller::PositionGoalChecker',
            'xy_goal_tolerance': 0.25,
            'stateful': True,
        },
        'FollowPath': {
            'plugin': 'nav2_colregs_alos_controller::ALOSController',
            'desired_linear_vel': 0.5,
            'max_linear_accel': 0.5,
            'max_angular_vel': 0.5,
            'max_angular_accel': 1.0,
            'forward_dist': 2.0,
            'gamma': 0.0006,
            'beta_hat0': 0.0,
            'reset_beta_on_new_goal': True,
            'beta_reset_goal_dist_tolerance': 0.05,
            'max_angle_for_motion': 1.047,
            'max_robot_pose_search_dist': 10.0,
        },
    }
    assert params['local_costmap']['local_costmap']['ros__parameters'] == {
        'update_frequency': 5.0,
        'publish_frequency': 2.0,
        'global_frame': 'odom',
        'robot_base_frame': 'base_link',
        'rolling_window': True,
        'width': 20,
        'height': 20,
        'resolution': 0.05,
        'robot_radius': 0.22,
        'plugins': ['voxel_layer', 'ts_projection_layer', 'inflation_layer'],
        'ts_projection_layer': {
            'plugin': 'nav2_colregs_costmap_layers::TSProjectionLayer',
            'enabled': True,
            'tracked_ship_topic': '/tracked_ship',
            'track_timeout': 3.0,
        },
        'inflation_layer': {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'cost_scaling_factor': 3.0,
            'inflation_radius': 0.70,
        },
        'voxel_layer': {
            'plugin': 'nav2_costmap_2d::VoxelLayer',
            'enabled': True,
            'publish_voxel_map': True,
            'origin_z': 0.0,
            'z_resolution': 0.05,
            'z_voxels': 16,
            'max_obstacle_height': 2.0,
            'mark_threshold': 0,
            'observation_sources': 'scan',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 6.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 6.0,
                'obstacle_min_range': 0.0,
                'observation_persistence': 0.2,
                'expected_update_rate': 5.0,
            },
        },
        'always_send_full_costmap': True,
        'introspection_mode': 'disabled',
    }


def test_bringup_declares_demo_runtime_dependencies():
    package = ET.parse(PACKAGE_DIR / 'package.xml')
    dependencies = {element.text for element in package.findall('./exec_depend')}

    assert {
        'ament_index_python',
        'launch',
        'launch_ros',
        'nav2_colregs_alos_controller',
        'nav2_colregs_costmap_layers',
        'nav2_colregs_local_path_behavior',
        'nav2_colregs_local_path_bt_nodes',
        'nav2_colregs_local_planner_server',
        'nav2_colregs_ts_manager',
        'nav2_lifecycle_manager',
    }.issubset(dependencies)


def test_obsolete_compute_local_path_contract_is_removed():
    source_dir = PACKAGE_DIR.parent
    msgs_dir = source_dir / 'nav2_colregs_msgs'
    msgs_cmake = (msgs_dir / 'CMakeLists.txt').read_text()
    msgs_manifest = (msgs_dir / 'package.xml').read_text()

    assert 'ComputeLocalPath.action' not in msgs_cmake
    assert 'nav_msgs' not in msgs_cmake
    assert 'ComputeLocalPath' not in msgs_manifest
    assert '<depend>nav_msgs</depend>' not in msgs_manifest
    assert not (msgs_dir / 'action' / 'ComputeLocalPath.action').exists()

    bt_dir = source_dir / 'nav2_colregs_local_path_bt_nodes'
    bt_cmake = (bt_dir / 'CMakeLists.txt').read_text()
    bt_manifest = (bt_dir / 'package.xml').read_text()

    for obsolete_reference in (
        'nav2_compute_local_path_action_bt_node',
        'compute_local_path_action.cpp',
        'test_compute_local_path_action',
        'nav2_colregs_msgs',
        'ament_cmake_gtest',
    ):
        assert obsolete_reference not in bt_cmake
    assert 'ComputeLocalPath' not in bt_manifest
    assert '<depend>nav2_colregs_msgs</depend>' not in bt_manifest
    assert '<test_depend>ament_cmake_gtest</test_depend>' not in bt_manifest

    for obsolete_file in (
        'include/nav2_colregs_local_path_bt_nodes/compute_local_path_action.hpp',
        'src/compute_local_path_action.cpp',
        'test/test_compute_local_path_action.cpp',
    ):
        assert not (bt_dir / obsolete_file).exists()

    assert 'nav2_create_local_path_action_bt_node' in bt_cmake
    assert 'src/create_local_path_action.cpp' in bt_cmake
    assert 'nav2_colregs_local_path_behavior' in bt_cmake
    assert 'nav2_colregs_local_path_behavior' in bt_manifest
    assert 'install(TARGETS\n  nav2_create_local_path_action_bt_node' in bt_cmake
    assert 'install(DIRECTORY include/' in bt_cmake
    assert 'ament_export_include_directories(include)' in bt_cmake
    assert 'ament_export_dependencies(${dependencies})' in bt_cmake
    create_header = (
        bt_dir
        / 'include/nav2_colregs_local_path_bt_nodes/create_local_path_action.hpp'
    )
    assert create_header.is_file()
    assert (bt_dir / 'src/create_local_path_action.cpp').is_file()


def test_local_planner_server_interrupts_for_cancel_or_preemption_before_publication():
    source = (
        PACKAGE_DIR.parent
        / 'nav2_colregs_local_planner_server'
        / 'src'
        / 'local_planner_server.cpp'
    ).read_text()

    assert 'create_publisher<nav_msgs::msg::Path>("plan", 1)' in source

    compute_plan = source[source.index('void ColregsLocalPlannerServer::computePlan()'):]
    first_cancel = compute_plan.index('if (current_canceled())')
    plan = compute_plan.index('const auto status = planner.planPath(')
    interruption_callback = compute_plan.index(
        'snapshot, interrupted, planning_deadline, nodes);', plan
    )
    success_gate = compute_plan.index(
        'if (status != PlanStatus::SUCCESS || nodes.empty())', plan
    )
    failure_return = compute_plan.index('return;', success_gate)
    atomic_success = compute_plan.index(
        'action_server_->succeed_current_if_not_interrupted(', failure_return
    )
    publish = compute_plan.index('plan_publisher_->publish', atomic_success)

    assert first_cancel < plan < interruption_callback
    assert interruption_callback < success_gate < failure_return
    assert failure_return < atomic_success < publish


def test_pending_goal_cancellation_is_handled_atomically():
    source_dir = PACKAGE_DIR.parent
    action_server = (
        source_dir / 'nav2_util' / 'include' / 'nav2_util'
        / 'simple_action_server.hpp'
    ).read_text()
    planner_server = (
        source_dir / 'nav2_colregs_local_planner_server' / 'src'
        / 'local_planner_server.cpp'
    ).read_text()

    assert 'bool terminate_pending_goal_if_cancel_requested()' in action_server
    assert 'terminate_pending_goal_if_cancel_requested()' in planner_server
    assert not (
        'is_pending_goal_cancel_requested()) {' in planner_server
        and 'terminate_pending_goal();' in planner_server
    )

    compute_plan = planner_server[
        planner_server.index('void ColregsLocalPlannerServer::computePlan()'):
    ]
    interruption_callback = compute_plan[
        compute_plan.index('auto interrupted = [this]() {'):
        compute_plan.index('};', compute_plan.index('auto interrupted = [this]() {'))
    ]
    dispose = interruption_callback.index(
        'terminate_pending_goal_if_cancel_requested()'
    )
    current_cancel = interruption_callback.index(
        'is_current_goal_cancel_requested()'
    )
    preempt = interruption_callback.index('is_preempt_requested()')
    assert dispose < current_cancel < preempt


def test_transformed_coordinates_are_checked_before_world_to_map():
    source = (
        PACKAGE_DIR.parent
        / 'nav2_colregs_local_planner_server'
        / 'src'
        / 'local_planner_server.cpp'
    ).read_text()
    compute_plan = source[source.index('void ColregsLocalPlannerServer::computePlan()'):]

    finite_check = compute_plan.index('std::isfinite(transformed_start.pose.position.x)')
    start_conversion = compute_plan.index('snapshot.worldToMap(')
    goal_conversion = compute_plan.index('snapshot.worldToMap(', start_conversion + 1)

    assert finite_check < start_conversion < goal_conversion
