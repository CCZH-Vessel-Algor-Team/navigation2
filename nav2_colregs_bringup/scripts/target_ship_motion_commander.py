#!/usr/bin/env python3
"""Drive target ship with ping-pong linear motion commands.

Switching logic is closed-loop on Gazebo world absolute pose, not /tracked_ship.
"""

import math
import queue
import re
import subprocess
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TargetShipMotionCommander(Node):
    def __init__(self):
        super().__init__('target_ship_motion_commander')

        self.declare_parameter('speed', 0.3)
        self.declare_parameter('pingpong_distance', 6.0)
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('reverse_cooldown_sec', 0.6)
        self.declare_parameter('reverse_rearm_distance', 0.5)
        self.declare_parameter('motion_axis_mode', 'auto')
        self.declare_parameter('motion_axis_angle_rad', 0.0)
        self.declare_parameter('gz_pose_topic', '/world/default/pose/info')
        self.declare_parameter('target_model_name', 'target_ship')

        self.speed = float(self.get_parameter('speed').value)
        self.pingpong_distance = float(self.get_parameter('pingpong_distance').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.reverse_cooldown_sec = float(self.get_parameter('reverse_cooldown_sec').value)
        self.reverse_rearm_distance = float(self.get_parameter('reverse_rearm_distance').value)
        self.motion_axis_mode = str(self.get_parameter('motion_axis_mode').value)
        self.motion_axis_angle_rad = float(self.get_parameter('motion_axis_angle_rad').value)
        self.gz_pose_topic = str(self.get_parameter('gz_pose_topic').value)
        self.target_model_name = str(self.get_parameter('target_model_name').value)

        self.leg_sign = 1.0
        self.leg_progress = 0.0
        self.last_reverse_time = self.get_clock().now()
        self.reverse_armed = True
        self.leg_start_pose = None
        self.axis_world = None
        self.latest_pose = None
        self._reverse_count = 0

        self._events = queue.Queue()
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_main, daemon=True)
        self._reader_thread.start()

        self.cmd_pub = self.create_publisher(Twist, '/target_ship/cmd_vel', 10)
        self.timer = self.create_timer(self.publish_period_sec, self._tick)

        self._name_regex = re.compile(r'^\s*name:\s+"([^"]+)"\s*$')
        self._x_regex = re.compile(r'^\s*x:\s+([-+0-9.eE]+)\s*$')
        self._y_regex = re.compile(r'^\s*y:\s+([-+0-9.eE]+)\s*$')
        self._z_regex = re.compile(r'^\s*z:\s+([-+0-9.eE]+)\s*$')
        self._w_regex = re.compile(r'^\s*w:\s+([-+0-9.eE]+)\s*$')

        self.get_logger().info(
            f'Motion commander started: speed={self.speed:.2f} m/s, '
            f'pingpong_distance={self.pingpong_distance:.2f} m, '
            f'axis_mode={self.motion_axis_mode}')

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

                m = self._name_regex.match(line)
                if m:
                    current_name = m.group(1)
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
                    mx = self._x_regex.match(line)
                    if mx:
                        current_x = float(mx.group(1))
                        continue
                    my = self._y_regex.match(line)
                    if my:
                        current_y = float(my.group(1))
                        continue
                    mz = self._z_regex.match(line)
                    if mz:
                        current_z = float(mz.group(1))
                        continue

                if in_orientation:
                    mx = self._x_regex.match(line)
                    if mx:
                        current_qx = float(mx.group(1))
                        continue
                    my = self._y_regex.match(line)
                    if my:
                        current_qy = float(my.group(1))
                        continue
                    mz = self._z_regex.match(line)
                    if mz:
                        current_qz = float(mz.group(1))
                        continue
                    mw = self._w_regex.match(line)
                    if mw:
                        current_qw = float(mw.group(1))
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

    def _init_axis_if_needed(self, pose):
        if self.axis_world is not None:
            return

        if self.motion_axis_mode == 'x':
            self.axis_world = (1.0, 0.0)
            return
        if self.motion_axis_mode == 'y':
            self.axis_world = (0.0, 1.0)
            return
        if self.motion_axis_mode == 'angle':
            self.axis_world = (
                math.cos(self.motion_axis_angle_rad),
                math.sin(self.motion_axis_angle_rad),
            )
            return

        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        self.axis_world = (math.cos(yaw), math.sin(yaw))

    def _tick(self):
        latest = None
        while not self._events.empty():
            latest = self._events.get_nowait()

        if latest is not None:
            self.latest_pose = latest
            if self.leg_start_pose is None:
                self.leg_start_pose = (latest[0], latest[1])
                self._init_axis_if_needed(latest)

        if self.latest_pose is not None and self.leg_start_pose is not None and self.axis_world is not None:
            dx = self.latest_pose[0] - self.leg_start_pose[0]
            dy = self.latest_pose[1] - self.leg_start_pose[1]
            projected = dx * self.axis_world[0] + dy * self.axis_world[1]
            self.leg_progress = abs(projected)

        now = self.get_clock().now()
        cooldown = (now - self.last_reverse_time).nanoseconds / 1e9
        if (self.reverse_armed and
                self.leg_progress >= self.pingpong_distance and
                cooldown >= self.reverse_cooldown_sec and
                self.latest_pose is not None):
            self.leg_sign *= -1.0
            self.last_reverse_time = now
            self.reverse_armed = False
            self.leg_start_pose = (self.latest_pose[0], self.latest_pose[1])
            self.leg_progress = 0.0
            self._reverse_count += 1
            self.get_logger().info(
                f'Target ship reached leg limit, reversing (count={self._reverse_count})')

        if (not self.reverse_armed and
                self.leg_progress >= self.reverse_rearm_distance):
            self.reverse_armed = True

        twist = Twist()
        twist.linear.x = self.speed * self.leg_sign
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main():
    rclpy.init()
    node = TargetShipMotionCommander()
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
