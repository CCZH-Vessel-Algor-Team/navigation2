"""Check real Costmap2DROS plugins in an isolated ROS domain, without navigation goals."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster
from usv_interfaces.msg import TrackedObstacle, TrackedObstacleList
import yaml


def test_two_costmaps_and_publishers(tmp_path, monkeypatch):
    """Verify serialized inputs, plugin lifecycle, TF, inflation and silent expiry.

    :param tmp_path: Pytest directory retaining generated parameters and node logs.
    :param monkeypatch: Pytest fixture for process-local environment isolation.
    """
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('PERCEIVED_TEST_DOMAIN_ID', '93'))
    monkeypatch.setenv('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    rclpy.init()
    node = rclpy.create_node('perceived_layer_integration')
    processes = []
    grids = {}
    log_paths = []
    positions = {'buoy': (5.0, 5.0, 0.8), 'storm': (15.0, 0.0, 3.0)}
    publishers = {kind: node.create_publisher(TrackedObstacleList, '/tracked_obstacles', 10)
                  for kind in positions}

    def publish():
        for number, (kind, position) in enumerate(positions.items(), 1):
            msg = TrackedObstacleList()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            obj = TrackedObstacle()
            obj.type = kind
            obj.target_id.uuid[0] = number
            obj.pose.position.x, obj.pose.position.y, obj.radius = position
            msg.obstacles = [obj]
            publishers[kind].publish(msg)

    def spin_until(predicate, timeout=10.0, publishing=False):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            assert all(proc.poll() is None for proc in processes), 'Costmap process exited'
            if publishing and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + 0.1
            rclpy.spin_once(node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError('Timed out waiting for costmap/lifecycle result')

    def transition(namespace, transition_id):
        client = node.create_client(ChangeState, f'/{namespace}/planner_server/change_state')
        try:
            assert client.wait_for_service(timeout_sec=10.0)
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = client.call_async(request)
            spin_until(future.done)
            assert future.result().success
        finally:
            node.destroy_client(client)

    def occupancy(namespace, x, y):
        grid = grids.get(namespace)
        if grid is None:
            return None
        if grid.header.frame_id == 'odom':
            x, y = (math.cos(0.3) * (x - 2.0) + math.sin(0.3) * (y - 1.0),
                    -math.sin(0.3) * (x - 2.0) + math.cos(0.3) * (y - 1.0))
        mx = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
        my = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= mx < grid.info.width and 0 <= my < grid.info.height):
            return None
        return grid.data[my * grid.info.width + mx]

    namespaces = ['perceived_global_test', 'perceived_local_test']
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for child in ['base_link', 'odom']:
            tf = TransformStamped()
            tf.header.frame_id = 'map'
            tf.child_frame_id = child
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.transform.rotation.w = 1.0
            if child == 'odom':
                tf.transform.translation.x = 2.0
                tf.transform.translation.y = 1.0
                tf.transform.rotation.z = math.sin(0.15)
                tf.transform.rotation.w = math.cos(0.15)
            transforms.append(tf)
        broadcaster.sendTransform(transforms)
        params = {}
        for namespace in namespaces:
            local = namespace == namespaces[1]
            params[f'/{namespace}/planner_server'] = {'ros__parameters': {
                'use_sim_time': False, 'planner_plugins': ['GridBased'],
                'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner'}}}
            params[f'/{namespace}/global_costmap/global_costmap'] = {'ros__parameters': {
                'use_sim_time': False, 'global_frame': 'odom' if local else 'map',
                'robot_base_frame': 'base_link', 'rolling_window': local,
                'width': 40 if local else 80, 'height': 40 if local else 80,
                'origin_x': -40.0, 'origin_y': -40.0,
                'resolution': 0.2 if local else 1.0,
                'update_frequency': 10.0, 'publish_frequency': 10.0,
                'track_unknown_space': False, 'robot_radius': 0.25,
                'always_send_full_costmap': True,
                'plugins': ['perceived_obstacle_layer', 'inflation_layer'],
                'perceived_obstacle_layer': {
                    'plugin': 'nav2_colregs_costmap_layers::PerceivedObstacleLayer',
                    'tracked_obstacle_topic': '/tracked_obstacles',
                    'observation_timeout': 1.0, 'tracking_frame': 'map'},
                'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                                    'inflation_radius': 2.0, 'cost_scaling_factor': 1.0}}}
            node.create_subscription(
                OccupancyGrid, f'/{namespace}/global_costmap/costmap',
                lambda msg, name=namespace: grids.__setitem__(name, msg), qos_profile_sensor_data)
        path = tmp_path / 'params.yaml'
        path.write_text(yaml.safe_dump(params))
        executable = Path(get_package_prefix('nav2_planner')) / 'lib/nav2_planner/planner_server'
        for namespace in namespaces:
            log_path = tmp_path / f'{namespace}.log'
            log_paths.append(log_path)
            with log_path.open('w') as output:
                processes.append(subprocess.Popen(
                    [str(executable), '--ros-args', '-r', f'__ns:=/{namespace}',
                     '--params-file', str(path)], stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True))
            transition(namespace, 1)
            transition(namespace, 3)

        spin_until(lambda: all(occupancy(ns, 5.0, 5.0) == 100 and
                               occupancy(ns, 15.0, 0.0) == 100 for ns in namespaces), publishing=True)
        print('PASS: two independent publishers marked both real costmaps (map and rotated odom).')
        # Inside the inflation band but outside the physical buoy on the finer local map.
        spin_until(lambda: occupancy(namespaces[1], 6.4, 5.0) is not None and
                   0 < occupancy(namespaces[1], 6.4, 5.0) < 100, publishing=True)
        positions['buoy'] = (-10.0, 5.0, 0.8)
        spin_until(lambda: all(occupancy(ns, -10.0, 5.0) == 100 and
                               occupancy(ns, 5.0, 5.0) == 0 and
                               occupancy(ns, 15.0, 0.0) == 100 for ns in namespaces), publishing=True)
        print('PASS: moved buoy withdrawn; independent storm retained; inflation enabled.')
        # Stop both publishers: no callback is needed to expire the final objects.
        spin_until(lambda: all(occupancy(ns, -10.0, 5.0) == 0 and
                               occupancy(ns, 15.0, 0.0) == 0 for ns in namespaces), timeout=5.0)
        print('PASS: silent input expiry cleared both costmaps, including inflation.')
        for namespace in namespaces:
            transition(namespace, 4)
            transition(namespace, 2)
    finally:
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        for path in log_paths:
            print(f'Costmap log: {path}\n{path.read_text()}')
