import ast
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition


PACKAGE_DIR = Path(__file__).parents[1]


def load_launch_module(name):
    launch_file = PACKAGE_DIR / 'launch' / name
    spec = spec_from_file_location(launch_file.stem, launch_file)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assigned_call(tree, variable_name):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == variable_name
            for target in node.targets
        ):
            return node.value
    raise AssertionError(f'No assignment found for {variable_name}')


def keyword(call, name):
    return next(item.value for item in call.keywords if item.arg == name)


def included_launch_filename(include_call):
    source = include_call.args[0]
    path = source.args[0]
    return ast.literal_eval(path.args[-1])


def test_launches_server_with_dedicated_lifecycle_manager():
    launch_file = PACKAGE_DIR / 'launch' / 'local_planner_server_launch.py'
    module = load_launch_module(launch_file.name)

    entities = module.generate_launch_description().entities
    declared_arguments = {
        entity.name
        for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert declared_arguments == {'params_file', 'use_sim_time', 'autostart'}
    assert sum(isinstance(entity, Node) for entity in entities) == 2

    tree = ast.parse(launch_file.read_text())
    node_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
    ]
    node_names = {ast.literal_eval(keyword(call, 'name')) for call in node_calls}
    assert node_names == {
        'colregs_local_planner_server',
        'lifecycle_manager_colregs_local_planner',
    }
    assert 'nav2_colregs_local_planner_server' in ast.unparse(node_calls[0])
    assert "'colregs_local_planner_server'" in ast.unparse(node_calls[1])


def test_primary_launch_gates_nav2_until_local_planner_is_active():
    launch_file = (
        PACKAGE_DIR / 'launch' / 'colregs_ts_projection_validation_launch.py'
    )
    module = load_launch_module(launch_file.name)
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)

    entities = module.generate_launch_description().entities
    lifecycle_nodes = [
        entity for entity in entities if isinstance(entity, LifecycleNode)
    ]
    handlers = [
        entity for entity in entities if isinstance(entity, RegisterEventHandler)
    ]
    top_level_includes = [
        entity for entity in entities if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(lifecycle_nodes) == 1
    assert sum(type(entity) is Node for entity in entities) == 1
    assert len(handlers) == 1
    assert isinstance(handlers[0].event_handler, OnStateTransition)
    assert len(handlers[0].event_handler.entities) == 1
    assert isinstance(
        handlers[0].event_handler.entities[0], IncludeLaunchDescription
    )
    assert len(top_level_includes) == 1

    tree = ast.parse(launch_file.read_text())
    server = assigned_call(tree, 'local_planner_server')
    assert isinstance(server.func, ast.Name) and server.func.id == 'LifecycleNode'
    assert ast.literal_eval(keyword(server, 'package')) == (
        'nav2_colregs_local_planner_server'
    )
    assert ast.literal_eval(keyword(server, 'executable')) == (
        'colregs_local_planner_server'
    )
    assert ast.literal_eval(keyword(server, 'name')) == (
        'colregs_local_planner_server'
    )
    assert ast.literal_eval(keyword(server, 'namespace')) == ''
    assert ast.unparse(keyword(server, 'parameters')) == (
        "[params_file, {'use_sim_time': use_sim_time}]"
    )

    manager = assigned_call(tree, 'lifecycle_manager_colregs_local_planner')
    assert isinstance(manager.func, ast.Name) and manager.func.id == 'Node'
    assert ast.literal_eval(keyword(manager, 'package')) == 'nav2_lifecycle_manager'
    assert ast.literal_eval(keyword(manager, 'executable')) == 'lifecycle_manager'
    assert ast.literal_eval(keyword(manager, 'name')) == (
        'lifecycle_manager_colregs_local_planner'
    )
    manager_parameters = ast.unparse(keyword(manager, 'parameters'))
    assert "'use_sim_time': use_sim_time" in manager_parameters
    assert "'autostart': autostart" in manager_parameters
    assert "'node_names': ['colregs_local_planner_server']" in manager_parameters

    state_handler = assigned_call(tree, 'start_base_on_local_planner_active')
    on_transition = state_handler.args[0]
    assert isinstance(on_transition.func, ast.Name)
    assert on_transition.func.id == 'OnStateTransition'
    assert ast.unparse(keyword(on_transition, 'target_lifecycle_node')) == (
        'local_planner_server'
    )
    assert ast.literal_eval(keyword(on_transition, 'goal_state')) == 'active'
    assert ast.unparse(keyword(on_transition, 'entities')) == '[base_launch]'
    transition_keywords = {item.arg: item.value for item in on_transition.keywords}
    handle_once = transition_keywords.get('handle_once', ast.Constant(value=False))
    assert ast.literal_eval(handle_once) is True

    base_launch = assigned_call(tree, 'base_launch')
    assert included_launch_filename(base_launch) == (
        'colregs_ts_simulation_launch.py'
    )
    base_arguments = ast.unparse(keyword(base_launch, 'launch_arguments'))
    assert "'params_file': params_file" in base_arguments
    assert "'use_sim_time': use_sim_time" in base_arguments
    assert "'autostart': autostart" in base_arguments
    assert "'headless': headless" in base_arguments

    ts_subsystem = assigned_call(tree, 'ts_subsystem')
    assert included_launch_filename(ts_subsystem) == 'ts_subsystem_launch.py'
    ts_arguments = ast.unparse(keyword(ts_subsystem, 'launch_arguments'))
    assert "'params_file': params_file" in ts_arguments
    assert "'use_sim_time': use_sim_time" in ts_arguments

    top_level_actions = {
        ast.unparse(node.args[0])
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'add_action'
    }
    assert 'base_launch' not in top_level_actions
    assert {
        'ts_subsystem',
        'local_planner_server',
        'start_base_on_local_planner_active',
        'lifecycle_manager_colregs_local_planner',
    }.issubset(top_level_actions)
