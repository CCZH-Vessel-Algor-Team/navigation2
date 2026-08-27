import ast
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


PACKAGE_DIR = Path(__file__).parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'


def load_launch_module(name):
    launch_file = LAUNCH_DIR / name
    spec = spec_from_file_location(launch_file.stem, launch_file)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_tree(name):
    launch_file = LAUNCH_DIR / name
    assert launch_file.exists(), f'Missing launch file: {launch_file.name}'
    return ast.parse(launch_file.read_text())


def calls_named(tree, name):
    return [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == name
    ]


def keyword(call, name):
    return next(item.value for item in call.keywords if item.arg == name)


def literal_keyword(call, name):
    return ast.literal_eval(keyword(call, name))


def call_by_name(calls, name):
    return next(call for call in calls if literal_keyword(call, 'name') == name)


def assigned_value(tree, variable_name):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == variable_name
            for target in node.targets
        ):
            return node.value
    raise AssertionError(f'No assignment found for {variable_name}')


def included_launch_filename(include_call):
    source = include_call.args[0]
    path = source.args[0]
    return ast.literal_eval(path.args[-1])


def declared_argument_names(tree):
    return {
        ast.literal_eval(call.args[0])
        for call in calls_named(tree, 'DeclareLaunchArgument')
    }


def declared_argument(tree, name):
    return next(
        call for call in calls_named(tree, 'DeclareLaunchArgument')
        if ast.literal_eval(call.args[0]) == name
    )


def declared_argument_defaults(tree):
    for call in calls_named(tree, 'DeclareLaunchArgument'):
        yield (
            ast.literal_eval(call.args[0]),
            ast.unparse(keyword(call, 'default_value')),
        )


def launch_arguments(call):
    return ast.unparse(keyword(call, 'launch_arguments'))


def effective_child_namespace(namespace, use_namespace):
    module = load_launch_module('colregs_bringup_launch.py')
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)
    group = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, GroupAction)
    )
    include = next(
        entity
        for entity in group.get_sub_entities()
        if isinstance(entity, IncludeLaunchDescription)
    )
    namespace_value = dict(include.launch_arguments)['namespace']
    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': namespace,
        'use_namespace': use_namespace,
    })
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(namespace_value))


def test_debug_launch_remains_an_isolated_entrypoint():
    module = load_launch_module('local_planner_server_launch.py')
    entities = module.generate_launch_description().entities
    declared_arguments = {
        entity.name
        for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert declared_arguments == {'params_file', 'use_sim_time', 'autostart'}
    assert sum(isinstance(entity, Node) for entity in entities) == 2

    tree = launch_tree('local_planner_server_launch.py')
    node_calls = calls_named(tree, 'Node')
    assert {literal_keyword(call, 'name') for call in node_calls} == {
        'colregs_local_planner_server',
        'lifecycle_manager_colregs_local_planner',
    }


def test_navigation_launch_has_exact_non_composed_topology():
    tree = launch_tree('colregs_navigation_launch.py')
    source = ast.unparse(tree)

    assert 'nav2_planner' not in source
    assert 'nav2_planner::PlannerServer' not in source
    assert 'global_costmap' not in source

    node_calls = calls_named(tree, 'Node')
    assert all(
        literal_keyword(call, 'name') != 'planner_server'
        for call in node_calls
    )
    assert {literal_keyword(call, 'name') for call in node_calls} == {
        'controller_server',
        'colregs_local_planner_server',
        'behavior_server',
        'bt_navigator',
        'lifecycle_manager_navigation',
    }

    expected_nodes = {
        'controller_server': ('nav2_controller', 'controller_server'),
        'colregs_local_planner_server': (
            'nav2_colregs_local_planner_server',
            'colregs_local_planner_server',
        ),
        'behavior_server': ('nav2_behaviors', 'behavior_server'),
        'bt_navigator': ('nav2_bt_navigator', 'bt_navigator'),
        'lifecycle_manager_navigation': (
            'nav2_lifecycle_manager',
            'lifecycle_manager',
        ),
    }
    for name, (package, executable) in expected_nodes.items():
        call = call_by_name(node_calls, name)
        assert literal_keyword(call, 'package') == package
        assert literal_keyword(call, 'executable') == executable

    for name in ('controller_server', 'behavior_server'):
        assert ast.unparse(keyword(call_by_name(node_calls, name), 'remappings')) == (
            'remappings'
        )

    manager = call_by_name(node_calls, 'lifecycle_manager_navigation')
    manager_parameters = ast.unparse(keyword(manager, 'parameters'))
    assert "'node_names': lifecycle_nodes" in manager_parameters
    assert ast.literal_eval(assigned_value(tree, 'lifecycle_nodes')) == [
        'controller_server',
        'colregs_local_planner_server',
        'behavior_server',
        'bt_navigator',
    ]


def test_navigation_launch_has_exact_composed_topology():
    tree = launch_tree('colregs_navigation_launch.py')
    component_calls = calls_named(tree, 'ComposableNode')

    assert {literal_keyword(call, 'name') for call in component_calls} == {
        'controller_server',
        'colregs_local_planner_server',
        'behavior_server',
        'bt_navigator',
        'lifecycle_manager_navigation',
    }
    expected_plugins = {
        'controller_server': 'nav2_controller::ControllerServer',
        'colregs_local_planner_server': (
            'nav2_colregs_local_planner_server::ColregsLocalPlannerServer'
        ),
        'behavior_server': 'behavior_server::BehaviorServer',
        'bt_navigator': 'nav2_bt_navigator::BtNavigator',
        'lifecycle_manager_navigation': (
            'nav2_lifecycle_manager::LifecycleManager'
        ),
    }
    for name, plugin in expected_plugins.items():
        call = call_by_name(component_calls, name)
        assert literal_keyword(call, 'plugin') == plugin

    for name in ('controller_server', 'behavior_server'):
        assert ast.unparse(
            keyword(call_by_name(component_calls, name), 'remappings')
        ) == 'remappings'

    load_calls = calls_named(tree, 'LoadComposableNodes')
    assert len(load_calls) == 1
    assert ast.unparse(keyword(load_calls[0], 'target_container')) == (
        'container_name_full'
    )


def test_custom_bringup_wraps_localization_and_minimal_navigation():
    tree = launch_tree('colregs_bringup_launch.py')
    source = ast.unparse(tree)
    assert 'nav2_planner' not in source
    assert 'PlannerServer' not in source
    assert 'global_costmap' not in source

    node_calls = calls_named(tree, 'Node')
    assert len(node_calls) == 1
    container = node_calls[0]
    assert literal_keyword(container, 'name') == 'nav2_container'
    assert literal_keyword(container, 'package') == 'rclcpp_components'
    assert literal_keyword(container, 'executable') == 'component_container_isolated'
    assert ast.unparse(keyword(container, 'condition')) == (
        'IfCondition(use_composition)'
    )

    includes = calls_named(tree, 'IncludeLaunchDescription')
    assert [included_launch_filename(call) for call in includes] == [
        'localization_launch.py',
        'colregs_navigation_launch.py',
    ]
    navigation_arguments = ast.unparse(keyword(includes[1], 'launch_arguments'))
    for argument in (
        'use_sim_time',
        'params_file',
        'autostart',
        'use_composition',
        'use_respawn',
    ):
        assert f"'{argument}': {argument}" in navigation_arguments
    assert "'namespace': effective_namespace" in navigation_arguments
    assert "'container_name': 'nav2_container'" in navigation_arguments


@pytest.mark.parametrize(
    ('namespace', 'use_namespace', 'expected'),
    [
        ('', 'false', ''),
        ('robot1', 'true', 'robot1'),
        ('robot1', 'false', ''),
    ],
)
def test_custom_bringup_normalizes_effective_namespace(
    namespace, use_namespace, expected
):
    assert effective_child_namespace(namespace, use_namespace) == expected

    tree = launch_tree('colregs_bringup_launch.py')
    effective_namespace = assigned_value(tree, 'effective_namespace')
    assert ast.unparse(effective_namespace) == (
        "IfElseSubstitution(use_namespace, if_value=namespace, else_value='')"
    )
    rewritten_yaml = calls_named(tree, 'RewrittenYaml')[0]
    assert ast.unparse(keyword(rewritten_yaml, 'root_key')) == 'effective_namespace'
    push_namespace = calls_named(tree, 'PushROSNamespace')[0]
    assert ast.unparse(keyword(push_namespace, 'namespace')) == 'effective_namespace'

    includes = calls_named(tree, 'IncludeLaunchDescription')
    for include in includes:
        assert "'namespace': effective_namespace" in launch_arguments(include)


def test_log_level_is_forwarded_across_all_bringup_edges():
    demo_tree = launch_tree('colregs_local_planner_demo_launch.py')
    assert 'log_level' in declared_argument_names(demo_tree)
    demo_includes = calls_named(demo_tree, 'IncludeLaunchDescription')
    assert "'log_level': log_level" in launch_arguments(demo_includes[0])

    # The restored legacy projection launch predates log-level forwarding; it
    # must stay faithful to its historical shape.
    legacy_tree = launch_tree('colregs_ts_projection_validation_launch.py')
    assert 'log_level' not in declared_argument_names(legacy_tree)

    simulation_tree = launch_tree('colregs_ts_simulation_launch.py')
    assert 'log_level' in declared_argument_names(simulation_tree)
    bringup = assigned_value(simulation_tree, 'bringup_cmd')
    assert "'log_level': log_level" in launch_arguments(bringup)

    bringup_tree = launch_tree('colregs_bringup_launch.py')
    assert 'log_level' in declared_argument_names(bringup_tree)
    container = call_by_name(calls_named(bringup_tree, 'Node'), 'nav2_container')
    assert ast.unparse(keyword(container, 'arguments')) == (
        "['--ros-args', '--log-level', log_level]"
    )
    child_includes = calls_named(bringup_tree, 'IncludeLaunchDescription')
    assert len(child_includes) == 2
    for include in child_includes:
        assert "'log_level': log_level" in launch_arguments(include)


def test_simulation_defaults_to_standard_bringup_and_accepts_override():
    tree = launch_tree('colregs_ts_simulation_launch.py')
    bringup_path = assigned_value(tree, 'bringup_launch_file')
    assert ast.unparse(bringup_path) == "LaunchConfiguration('bringup_launch_file')"

    declarations = calls_named(tree, 'DeclareLaunchArgument')
    declaration = next(
        call for call in declarations
        if ast.literal_eval(call.args[0]) == 'bringup_launch_file'
    )
    assert ast.unparse(keyword(declaration, 'default_value')) == (
        "os.path.join(nav2_launch_dir, 'bringup_launch.py')"
    )

    bringup = assigned_value(tree, 'bringup_cmd')
    assert ast.unparse(bringup.args[0].args[0]) == 'bringup_launch_file'


def test_rviz_defaults_keep_base_generic_and_demo_colregs_specific():
    simulation_tree = launch_tree('colregs_ts_simulation_launch.py')
    simulation_rviz = declared_argument(simulation_tree, 'rviz_config_file')
    assert ast.unparse(keyword(simulation_rviz, 'default_value')) == (
        "os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')"
    )

    demo_tree = launch_tree('colregs_local_planner_demo_launch.py')
    demo_rviz = declared_argument(demo_tree, 'rviz_config_file')
    assert ast.unparse(keyword(demo_rviz, 'default_value')) == (
        "os.path.join(bringup_dir, 'rviz', "
        "'colregs_local_planner_demo.rviz')"
    )

    # The restored legacy launch never declared an rviz override; it uses the
    # simulation default (nav2_default_view.rviz).
    legacy_tree = launch_tree('colregs_ts_projection_validation_launch.py')
    assert 'rviz_config_file' not in declared_argument_names(legacy_tree)


def test_projection_validation_restores_legacy_ts_subsystem_topology():
    tree = launch_tree('colregs_ts_projection_validation_launch.py')
    source = ast.unparse(tree)
    assert 'LifecycleNode' not in source
    assert 'OnStateTransition' not in source
    assert 'RegisterEventHandler' not in source
    assert 'lifecycle_manager_colregs_local_planner' not in source

    includes = calls_named(tree, 'IncludeLaunchDescription')
    assert [included_launch_filename(call) for call in includes] == [
        'colregs_ts_simulation_launch.py',
        'ts_subsystem_launch.py',
    ]
    simulation_arguments = ast.unparse(keyword(includes[0], 'launch_arguments'))
    for argument in ('params_file', 'use_sim_time', 'autostart', 'headless'):
        assert f"'{argument}': {argument}" in simulation_arguments
    # The legacy stack runs the standard bringup (planner_server chain); no
    # custom bringup injection and no colregs server nodes.
    assert 'bringup_launch_file' not in simulation_arguments

    ts_arguments = ast.unparse(keyword(includes[1], 'launch_arguments'))
    assert "'params_file': params_file" in ts_arguments
    assert "'use_sim_time': use_sim_time" in ts_arguments


def test_local_planner_demo_launch_wires_server_stack_and_open_world():
    tree = launch_tree('colregs_local_planner_demo_launch.py')
    source = ast.unparse(tree)
    assert 'ts_subsystem' not in source
    assert 'ts_state_manager' not in source
    assert 'LifecycleNode' not in source

    includes = calls_named(tree, 'IncludeLaunchDescription')
    assert [included_launch_filename(call) for call in includes] == [
        'colregs_ts_simulation_launch.py',
    ]
    simulation_arguments = ast.unparse(keyword(includes[0], 'launch_arguments'))
    assert "'bringup_launch_file': colregs_bringup_launch_file" in (
        simulation_arguments
    )
    for argument in (
        'world', 'map', 'x_pose', 'y_pose', 'yaw',
        'ts_x_pose', 'ts_y_pose', 'ts_yaw', 'ts_speed', 'ts_pingpong_distance',
        'ts_motion_axis_mode',
    ):
        assert f"'{argument}': {argument}" in simulation_arguments

    defaults = dict(declared_argument_defaults(tree))
    assert 'nav2_colregs_params_local_planner_demo.yaml' in defaults['params_file']
    assert 'colregs_open_world.sdf.xacro' in defaults['world']
    assert 'colregs_open_map.yaml' in defaults['map']
    assert (defaults['x_pose'], defaults['y_pose'], defaults['yaw']) == (
        "'6.00'", "'0.00'", "'0.00'")
    assert (defaults['ts_x_pose'], defaults['ts_y_pose']) == (
        "'8.00'", "'9.00'")
    assert defaults['ts_speed'] == "'0.4'"
    assert defaults['ts_pingpong_distance'] == "'10.0'"
    assert defaults['ts_motion_axis_mode'] == "'y'"
    assert "'ts_motion_axis_mode': ts_motion_axis_mode" in simulation_arguments
