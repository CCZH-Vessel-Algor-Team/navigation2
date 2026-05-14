#!/usr/bin/env python3
"""Publish /tracked_ship and TS TF from Gazebo world absolute pose."""

import queue
import re
import subprocess
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_colregs_msgs.msg import TrackedShip
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TargetShipStatePublisher(Node):
    def __init__(self):
        super().__init__('target_ship_state_publisher')

        self.declare_parameter('gz_pose_topic', '/world/default/pose/info')
        self.declare_parameter('target_model_name', 'target_ship')
        self.declare_parameter('tracked_topic', '/tracked_ship')
        self.declare_parameter('tracked_frame_id', 'world')
        self.declare_parameter('target_radius', 0.3)
        self.declare_parameter('tf_child_frame_id', 'ts_virtual_base_link')
        self.gz_pose_topic = str(self.get_parameter('gz_pose_topic').value)
        self.target_model_name = str(self.get_parameter('target_model_name').value)
        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.tracked_frame_id = str(self.get_parameter('tracked_frame_id').value)
        self.target_radius = float(self.get_parameter('target_radius').value)
        self.tf_child_frame_id = str(self.get_parameter('tf_child_frame_id').value)

        self._events = queue.Queue()
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_main, daemon=True)
        self._reader_thread.start()
        self.timer = self.create_timer(0.05, self._drain_events)

        self.tracked_pub = self.create_publisher(TrackedShip, self.tracked_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._count = 0
        self._name_regex = re.compile(r'^\s*name:\s+"([^"]+)"\s*$')
        self._x_regex = re.compile(r'^\s*x:\s+([-+0-9.eE]+)\s*$')
        self._y_regex = re.compile(r'^\s*y:\s+([-+0-9.eE]+)\s*$')
        self._z_regex = re.compile(r'^\s*z:\s+([-+0-9.eE]+)\s*$')
        self._w_regex = re.compile(r'^\s*w:\s+([-+0-9.eE]+)\s*$')

        self.get_logger().info(
            f'Listening Gazebo pose topic: {self.gz_pose_topic} for model={self.target_model_name}; '
            f'publishing {self.tracked_topic} and TF {self.tracked_frame_id}->{self.tf_child_frame_id}')

    def _reader_main(self):
        process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', self.gz_pose_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        current_name = None
        current_x = None
        current_y = None
        current_z = None
        current_qx = None
        current_qy = None
        current_qz = None
        current_qw = None
        in_position = False
        in_orientation = False

        try:
            for line in process.stdout:
                if self._stop_event.is_set():
                    break

                name_match = self._name_regex.match(line)
                if name_match:
                    current_name = name_match.group(1)
                    current_x = None
                    current_y = None
                    current_z = None
                    current_qx = None
                    current_qy = None
                    current_qz = None
                    current_qw = None
                    in_position = False
                    in_orientation = False
                    continue

                token = line.strip()
                if token == 'position {':
                    in_position = True
                    continue
                if token == 'orientation {':
                    in_orientation = True
                    continue

                if in_position:
                    x_match = self._x_regex.match(line)
                    if x_match:
                        current_x = float(x_match.group(1))
                        continue
                    y_match = self._y_regex.match(line)
                    if y_match:
                        current_y = float(y_match.group(1))
                        continue
                    z_match = self._z_regex.match(line)
                    if z_match:
                        current_z = float(z_match.group(1))
                        continue

                if in_orientation:
                    x_match = self._x_regex.match(line)
                    if x_match:
                        current_qx = float(x_match.group(1))
                        continue
                    y_match = self._y_regex.match(line)
                    if y_match:
                        current_qy = float(y_match.group(1))
                        continue
                    z_match = self._z_regex.match(line)
                    if z_match:
                        current_qz = float(z_match.group(1))
                        continue
                    w_match = self._w_regex.match(line)
                    if w_match:
                        current_qw = float(w_match.group(1))
                        continue

                if token == '}' and in_position:
                    in_position = False
                    continue

                if token == '}' and in_orientation:
                    in_orientation = False
                    if (current_name == self.target_model_name and
                            current_x is not None and current_y is not None and
                            current_qx is not None and current_qy is not None and
                            current_qz is not None and current_qw is not None):
                        self._events.put((
                            current_x,
                            current_y,
                            current_z if current_z is not None else 0.0,
                            current_qx,
                            current_qy,
                            current_qz,
                            current_qw,
                        ))
        finally:
            if process.poll() is None:
                process.terminate()

    def _drain_events(self):
        latest = None
        while not self._events.empty():
            latest = self._events.get_nowait()

        if latest is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._count += 1

        tracked = TrackedShip()
        tracked.header.stamp = stamp
        tracked.header.frame_id = self.tracked_frame_id
        tracked.pose.position.x = latest[0]
        tracked.pose.position.y = latest[1]
        tracked.pose.position.z = latest[2]
        tracked.pose.orientation.x = latest[3]
        tracked.pose.orientation.y = latest[4]
        tracked.pose.orientation.z = latest[5]
        tracked.pose.orientation.w = latest[6]
        tracked.radius = self.target_radius
        self.tracked_pub.publish(tracked)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.tracked_frame_id
        transform.child_frame_id = self.tf_child_frame_id
        transform.transform.translation.x = latest[0]
        transform.transform.translation.y = latest[1]
        transform.transform.translation.z = latest[2]
        transform.transform.rotation.x = latest[3]
        transform.transform.rotation.y = latest[4]
        transform.transform.rotation.z = latest[5]
        transform.transform.rotation.w = latest[6]
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(
            f'target_ship world pose sample#{self._count}: '
            f'x={latest[0]:.3f}, y={latest[1]:.3f}, z={latest[2]:.3f}',
            throttle_duration_sec=0.5)

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main():
    rclpy.init()
    node = TargetShipStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
