# Copyright 2026 vectorwang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Static and pure tests for the maritime situation monitor ROS boundary."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from nav2_maritime_situation_monitor.enu_algorithms import Vector2
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
NODE_PATH = (
    PACKAGE_ROOT
    / 'nav2_maritime_situation_monitor'
    / 'situation_monitor_node.py'
)
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'maritime_situation_monitor.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'maritime_situation_monitor.launch.py'
MESSAGE_ROOT = PACKAGE_ROOT.parent / 'nav2_maritime_situation_msgs' / 'msg'
README_PATH = PACKAGE_ROOT.parent / 'README.md'

APPROVED_DEFAULTS = {
    'publish_frequency': 0.5,
    'tracked_ship_topic': '/tracked_ship',
    'odom_topic': '/odom',
    'output_topic': '/maritime_situation',
    'global_frame': 'map',
    'base_frame': 'base_link',
    'target_timeout': 3.0,
    'ownship_timeout': 1.0,
    'transform_timeout': 0.2,
    'relative_speed_epsilon': 1.0e-6,
    'course_speed_epsilon': 0.05,
    'info_dcpa_threshold': 30.0,
    'info_tcpa_threshold': 120.0,
    'warning_dcpa_threshold': 20.0,
    'warning_tcpa_threshold': 30.0,
    'critical_dcpa_threshold': 10.0,
    'critical_tcpa_threshold': 10.0,
    'head_on_bearing_threshold_deg': 6.0,
    'reciprocal_heading_tolerance_deg': 15.0,
    'overtaking_stern_sector_deg': 112.5,
    'enable_markers': True,
    'marker_topic': '/maritime_situation_markers',
    'marker_scale': 3.0,
    'marker_label_scale': 2.0,
    'marker_velocity_scale': 2.0,
    'marker_line_width': 0.2,
    'marker_z_offset': 0.5,
}


def _node_tree():
    return ast.parse(NODE_PATH.read_text(encoding='utf-8'))


def _declared_parameters(tree):
    parameters = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr != 'declare_parameter' or len(call.args) < 2:
            continue
        parameters[ast.literal_eval(call.args[0])] = ast.literal_eval(call.args[1])
    return parameters


def _load_tf_helpers():
    tree = _node_tree()
    helper_names = {'yaw_from_quaternion', 'transform_point', 'rotate_vector'}
    helpers = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in helper_names
    ]
    assert {helper.name for helper in helpers} == helper_names
    module = ast.Module(body=helpers, type_ignores=[])
    namespace = {'math': math, 'Vector2': Vector2}
    exec(compile(ast.fix_missing_locations(module), str(NODE_PATH), 'exec'), namespace)
    return namespace


def _load_freshness_helpers():
    tree = _node_tree()
    helper_names = {'effective_source_time_ns', 'snapshot_is_stale'}
    helpers = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in helper_names
    ]
    assert {helper.name for helper in helpers} == helper_names
    module = ast.Module(body=helpers, type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), str(NODE_PATH), 'exec'), namespace)
    return namespace


def _transform(yaw, tx=0.0, ty=0.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=tx, y=ty),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
    )


def test_package_metadata_and_dependencies():
    root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert root.attrib['format'] == '3'
    assert root.findtext('name') == 'nav2_maritime_situation_monitor'
    assert root.findtext('license') == 'Apache-2.0'
    dependencies = {element.text for element in root.findall('exec_depend')}
    assert {
        'rclpy',
        'nav_msgs',
        'nav2_colregs_msgs',
        'ament_index_python',
        'launch',
        'launch_ros',
        'nav2_maritime_situation_msgs',
        'tf2_ros',
        'unique_identifier_msgs',
    } <= dependencies
    assert 'nav2_common' not in dependencies
    assert {
        element.text for element in root.findall('buildtool_depend')
    } == {'ament_python'}
    assert {
        element.text for element in root.findall('test_depend')
    } == {
        'ament_copyright',
        'ament_flake8',
        'ament_pep257',
        'python3-pytest',
        'python3-yaml',
    }
    assert root.findtext('export/build_type') == 'ament_python'


def test_setup_installs_ros_assets_and_console_script():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert "package_name = 'nav2_maritime_situation_monitor'" in setup_source
    assert "glob('config/*.yaml')" in setup_source
    assert "glob('launch/*.launch.py')" in setup_source
    assert 'maritime_situation_monitor = ' in setup_source
    assert 'nav2_maritime_situation_monitor.situation_monitor_node:main' in setup_source


def test_approved_parameter_contract_is_declared():
    assert _declared_parameters(_node_tree()) == APPROVED_DEFAULTS


def test_default_yaml_contains_exactly_approved_parameters():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    assert config == {'/**': {'ros__parameters': APPROVED_DEFAULTS}}


def test_standalone_launch_contract():
    source = LAUNCH_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    declared_arguments = {
        ast.literal_eval(call.args[0])
        for call in ast.walk(tree)
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'DeclareLaunchArgument'
        )
    }
    nodes = [
        call
        for call in ast.walk(tree)
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'Node'
        )
    ]

    assert declared_arguments == {'namespace', 'params_file', 'use_sim_time'}
    assert len(nodes) == 1
    keywords = {keyword.arg: ast.unparse(keyword.value) for keyword in nodes[0].keywords}
    assert ast.literal_eval(nodes[0].keywords[0].value) == (
        'nav2_maritime_situation_monitor'
    )
    assert keywords['executable'] == "'maritime_situation_monitor'"
    assert keywords['name'] == "'maritime_situation_monitor'"
    assert keywords['namespace'] == 'namespace'
    assert keywords['output'] == "'screen'"
    assert keywords['parameters'] == (
        "[params_file, {'use_sim_time': use_sim_time}]"
    )
    assert 'RewrittenYaml' not in source
    assert 'ParameterFile' not in source
    assert 'nav2_common' not in source
    assert 'IncludeLaunchDescription' not in source
    assert 'nav2_lifecycle_manager' not in source
    assert 'nav2_bringup' not in source
    assert 'nav2_colregs_ts' not in source


def test_monitor_is_not_managed_by_a_nav2_lifecycle_manager():
    for launch_path in PACKAGE_ROOT.parent.glob('**/*.launch.py'):
        source = launch_path.read_text(encoding='utf-8')
        if 'nav2_lifecycle_manager' in source or 'lifecycle_manager' in source:
            assert 'maritime_situation_monitor' not in source


def test_node_uses_pure_builder_and_parameterized_topics():
    source = NODE_PATH.read_text(encoding='utf-8')
    tree = _node_tree()
    imports = {
        alias.name
        for node in ast.walk(tree)
        if (
            isinstance(node, ast.ImportFrom)
            and node.level == 1
            and node.module == 'report_builder'
        )
        for alias in node.names
    }
    assert {'AssessmentConfig', 'TrackedMotion', 'build_reports'} <= imports
    assert 'compute_cpa' not in source
    assert 'classify_risk' not in source
    assert 'classify_encounter' not in source
    assert 'self._tracked_ship_topic' in source
    assert 'self._odom_topic' in source
    assert 'self._output_topic' in source


def test_qos_and_ros_boundary_contracts_are_explicit():
    source = NODE_PATH.read_text(encoding='utf-8')
    assert source.count('qos_profile_sensor_data') >= 3
    assert 'QoSReliabilityPolicy.RELIABLE' in source
    assert 'QoSDurabilityPolicy.VOLATILE' in source
    assert 'QoSHistoryPolicy.KEEP_LAST' in source
    assert 'depth=10' in source
    assert 'threading.Lock()' in source
    assert source.count('build_reports(') == 1
    assert 'add_on_set_parameters_callback' not in source
    assert 'LifecycleNode' not in source


def test_executor_allows_tf_callbacks_during_synchronous_waits():
    source = NODE_PATH.read_text(encoding='utf-8')
    assert 'MultiThreadedExecutor(num_threads=2)' in source
    assert 'MutuallyExclusiveCallbackGroup()' in source
    assert 'callback_group=self._timer_callback_group' in source
    assert 'isinstance(self._tf_listener.group, ReentrantCallbackGroup)' in source
    assert 'executor.add_node(node)' in source
    assert 'executor.spin()' in source
    assert 'executor.shutdown()' in source
    assert 'node.destroy_node()' in source
    assert 'rclpy.shutdown()' in source


def test_target_snapshot_uses_one_shared_frame_transform():
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    methods = {
        node.name: node for node in monitor.body if isinstance(node, ast.FunctionDef)
    }

    def lookup_calls(method):
        return [
            call
            for call in ast.walk(method)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == '_lookup_transform'
        ]

    assert lookup_calls(methods['_target_motion']) == []
    assess_lookups = lookup_calls(methods['_assess'])
    assert len(assess_lookups) == 1
    assert ast.unparse(assess_lookups[0].args[0]) == 'target_list.header.frame_id'
    assert ast.unparse(assess_lookups[0].args[1]) == 'target_source_time_ns'

    target_calls = [
        call
        for call in ast.walk(methods['_assess'])
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == '_target_motion'
    ]
    assert len(target_calls) == 1
    assert ast.unparse(target_calls[0].args[1]) == 'target_transform'


def test_lookup_transform_uses_explicit_source_time_not_latest():
    source = NODE_PATH.read_text(encoding='utf-8')
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    lookup = next(
        node
        for node in monitor.body
        if isinstance(node, ast.FunctionDef) and node.name == '_lookup_transform'
    )
    calls = [
        call
        for call in ast.walk(lookup)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'lookup_transform'
    ]
    assert len(calls) == 1
    assert [ast.unparse(arg) for arg in calls[0].args[:3]] == [
        'self._global_frame',
        'source_frame',
        'Time(nanoseconds=source_time_ns, clock_type=self.get_clock().clock_type)',
    ]
    assert 'Time()' not in source


def test_ownship_pose_and_twist_use_odom_source_time():
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    ownship_motion = next(
        node
        for node in monitor.body
        if isinstance(node, ast.FunctionDef) and node.name == '_ownship_motion'
    )
    calls = [
        call
        for call in ast.walk(ownship_motion)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == '_lookup_transform'
    ]
    assert len(calls) == 2
    assert {ast.unparse(call.args[0]) for call in calls} == {
        'odom.header.frame_id',
        'twist_frame',
    }
    assert all(ast.unparse(call.args[1]) == 'source_time_ns' for call in calls)


def test_assessment_propagates_each_snapshot_to_output_stamp():
    source = NODE_PATH.read_text(encoding='utf-8')
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    assess = next(
        node
        for node in monitor.body
        if isinstance(node, ast.FunctionDef) and node.name == '_assess'
    )
    calls = [
        call
        for call in ast.walk(assess)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'propagate_motion'
    ]
    assert 'odom_source_time_ns = effective_source_time_ns(*odom_snapshot)' in source
    assert 'target_source_time_ns = effective_source_time_ns(*target_snapshot)' in source
    assert len(calls) == 2
    assert {ast.unparse(call.args[1]) for call in calls} == {
        '(now.nanoseconds - odom_source_time_ns) / 1000000000.0',
        '(now.nanoseconds - target_source_time_ns) / 1000000000.0',
    }
    assert 'output.header.stamp = now.to_msg()' in source


def test_assessment_snapshots_inputs_before_capturing_evaluation_time():
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    assess = next(
        node
        for node in monitor.body
        if isinstance(node, ast.FunctionDef) and node.name == '_assess'
    )
    snapshot_assignment = assess.body[0]
    assert isinstance(snapshot_assignment, ast.Assign)
    assert len(snapshot_assignment.targets) == 1
    target = snapshot_assignment.targets[0]
    assert isinstance(target, ast.Tuple)
    assert [element.id for element in target.elts] == [
        'odom_snapshot',
        'target_snapshot',
    ]
    assert ast.unparse(snapshot_assignment.value) == 'self._snapshot_inputs()'
    assert ast.unparse(assess.body[1]) == 'now = self.get_clock().now()'
    assert ast.unparse(assess.body[2]) == 'output = self._empty_output(now)'


def test_propagation_value_errors_follow_existing_output_isolation_policy():
    tree = _node_tree()
    monitor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MaritimeSituationMonitor'
    )
    assess = next(
        node
        for node in monitor.body
        if isinstance(node, ast.FunctionDef) and node.name == '_assess'
    )
    ownship_try = next(node for node in assess.body if isinstance(node, ast.Try))
    target_loop = next(node for node in assess.body if isinstance(node, ast.For))
    target_try = next(node for node in target_loop.body if isinstance(node, ast.Try))

    ownship_handler = ast.unparse(ownship_try.handlers[0])
    target_handler = ast.unparse(target_try.handlers[0])
    assert 'ValueError' in ownship_handler
    assert 'self._publisher.publish(output)' in ownship_handler
    assert 'return' in ownship_handler
    assert 'ValueError' in target_handler
    assert 'return' not in target_handler
    assert 'targets.append(target)' in ast.unparse(target_try)


def test_readme_documents_strict_epsilon_boundary():
    readme = README_PATH.read_text(encoding='utf-8')
    assert '相对速度严格小于 `relative_speed_epsilon`' in readme
    assert '相对速度小于或等于 `relative_speed_epsilon`' not in readme


def _message(stamp_sec=0, stamp_nanosec=0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nanosec)
        )
    )


def test_zero_source_stamp_falls_back_to_receipt_time():
    helper = _load_freshness_helpers()['effective_source_time_ns']
    assert helper(_message(), 123) == 123
    assert helper(_message(4, 5), 123) == 4_000_000_005


@pytest.mark.parametrize(
    'snapshot',
    [
        (_message(11, 0), 9_000_000_000),
        (_message(9, 0), 11_000_000_000),
        (_message(), 11_000_000_000),
    ],
)
def test_future_source_or_receipt_timestamp_is_stale(snapshot):
    helper = _load_freshness_helpers()['snapshot_is_stale']
    assert helper(snapshot, 10_000_000_000, 3.0)


def test_freshness_timeout_uses_effective_source_time():
    helper = _load_freshness_helpers()['snapshot_is_stale']
    assert not helper((_message(8, 0), 9_000_000_000), 10_000_000_000, 3.0)
    assert helper((_message(6, 0), 9_000_000_000), 10_000_000_000, 3.0)


def _message_declarations(path):
    return [
        line.strip()
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def test_situation_report_interface_fields_and_constants_are_unchanged():
    assert _message_declarations(MESSAGE_ROOT / 'SituationReport.msg') == [
        'uint8 RISK_SAFE=0',
        'uint8 RISK_INFO=1',
        'uint8 RISK_WARNING=2',
        'uint8 RISK_CRITICAL=3',
        'uint8 ENCOUNTER_UNKNOWN=0',
        'uint8 ENCOUNTER_HEAD_ON=1',
        'uint8 ENCOUNTER_OVERTAKING=2',
        'uint8 ENCOUNTER_CROSSING_LEFT=3',
        'uint8 ENCOUNTER_CROSSING_RIGHT=4',
        'unique_identifier_msgs/UUID target_id',
        'bool cpa_valid',
        'float64 dcpa',
        'float64 tcpa',
        'uint8 encounter_type',
        'uint8 risk_level',
    ]


def test_message_comments_document_units_enums_and_header_semantics():
    report = (MESSAGE_ROOT / 'SituationReport.msg').read_text(encoding='utf-8')
    array = (MESSAGE_ROOT / 'SituationReportArray.msg').read_text(encoding='utf-8')
    assert 'meters' in report
    assert 'seconds' in report
    assert 'encounter_type must be one of ENCOUNTER_' in report
    assert 'risk_level must be one of RISK_' in report
    assert 'cpa_valid=false' in report
    assert 'tcpa=0, dcpa=current range, risk_level=RISK_SAFE' in report
    assert 'common evaluation time' in array
    assert 'global ENU assessment frame' in array


def test_tf_helpers_apply_translation_only_to_points():
    helpers = _load_tf_helpers()
    transform = _transform(math.pi / 2.0, tx=10.0, ty=-2.0)
    assert helpers['yaw_from_quaternion'](transform.transform.rotation) == pytest.approx(
        math.pi / 2.0
    )
    point = helpers['transform_point'](2.0, 0.0, transform)
    vector = helpers['rotate_vector'](2.0, 0.0, transform)
    assert (point.x, point.y) == pytest.approx((10.0, 0.0))
    assert (vector.x, vector.y) == pytest.approx((0.0, 2.0))


@pytest.mark.parametrize('helper', ['transform_point', 'rotate_vector'])
def test_tf_helpers_reject_nonfinite_values(helper):
    helpers = _load_tf_helpers()
    with pytest.raises(ValueError, match='finite'):
        helpers[helper](math.nan, 0.0, _transform(0.0))
    with pytest.raises(ValueError, match='finite'):
        helpers[helper](0.0, 0.0, _transform(0.0, tx=math.inf))


@pytest.mark.parametrize('helper', ['transform_point', 'rotate_vector'])
def test_tf_helpers_reject_nonfinite_results(helper):
    helpers = _load_tf_helpers()
    if helper == 'transform_point':
        transform = _transform(0.0, tx=1.5e308)
        values = (1.5e308, 0.0)
    else:
        transform = _transform(math.pi / 4.0)
        values = (1.5e308, -1.5e308)
    with pytest.raises(ValueError, match='finite'):
        helpers[helper](*values, transform)
