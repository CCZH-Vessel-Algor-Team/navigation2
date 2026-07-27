from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]


def test_demo_tree_computes_and_follows_local_path():
    tree = ET.parse(
        PACKAGE_DIR
        / 'behavior_trees'
        / 'navigate_w_colregs_local_planner_server.xml'
    )

    compute_local_path = tree.find('.//ComputeLocalPath')
    assert compute_local_path is not None
    assert compute_local_path.attrib == {
        'reference_path': '{path}',
        'local_path': '{local_path}',
        'error_code_id': '{compute_local_path_error_code}',
        'error_msg': '{compute_local_path_error_msg}',
    }
    assert tree.find('.//ComputeLocalPath/..').attrib == {
        'hz': '1.0',
        'name': 'RateControllerComputeLocalPath',
    }

    follow_path = tree.find('.//FollowPath')
    assert follow_path.attrib['path'] == '{local_path}'


def test_projection_validation_parameters_select_demo_tree_and_keep_fallback():
    params_file = (
        PACKAGE_DIR / 'params' / 'nav2_colregs_params_ts_projection_validation.yaml'
    )
    params = yaml.safe_load(params_file.read_text())
    bt_params = params['bt_navigator']['ros__parameters']

    assert 'nav2_compute_local_path_action_bt_node' in bt_params['plugin_lib_names']
    assert bt_params['default_nav_to_pose_bt_xml'].endswith(
        '/behavior_trees/navigate_w_colregs_local_planner_server.xml'
    )
    assert 'compute_local_path' in bt_params['error_code_name_prefixes']
    assert params['colregs_local_planner_server']['ros__parameters'] == {
        'use_sim_time': True,
        'action_server_result_timeout': 10.0,
    }

    behavior_params = params['behavior_server']['ros__parameters']
    assert 'create_local_path' in behavior_params['behavior_plugins']
    assert behavior_params['create_local_path']['plugin'] == (
        'nav2_colregs_local_path_behavior::CreateLocalPath'
    )


def test_bringup_declares_demo_runtime_dependencies():
    package = ET.parse(PACKAGE_DIR / 'package.xml')
    dependencies = {element.text for element in package.findall('./exec_depend')}

    assert {
        'ament_index_python',
        'launch',
        'launch_ros',
        'nav2_colregs_local_path_bt_nodes',
        'nav2_colregs_local_planner_server',
        'nav2_lifecycle_manager',
    }.issubset(dependencies)


def test_local_planner_server_has_reviewed_cancellation_checkpoints_and_log():
    source = (
        PACKAGE_DIR.parent
        / 'nav2_colregs_local_planner_server'
        / 'src'
        / 'local_planner_server.cpp'
    ).read_text()

    assert source.count('if (finalize_cancellation())') == 3
    final_check = source.rindex('if (finalize_cancellation())')
    publish = source.index('path_publisher_->publish', final_check)
    succeed = source.index('action_server_->succeeded_current', publish)
    assert final_check < publish < succeed
    assert 'reference poses' in source
    assert 'result poses' in source
    assert "frame '%s'" in source
