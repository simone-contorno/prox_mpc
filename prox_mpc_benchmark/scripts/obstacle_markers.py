#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Visualization-only ground-truth obstacle bodies for the demo recordings.

Publishes a visualization_msgs/MarkerArray on 'obstacle_bodies' with one cylinder
per scenario obstacle at its exact analytic position (mirrors obstacle_field.hpp
obstacleCenterAt, the same law the scan simulator uses), so the drawn body lines up
with the laser returns and the costmap. The obstacle clock is anchored to the
robot's first motion, identical to scan_simulator / gt_obstacle_publisher, so a
moving obstacle's phase matches the driven trajectory.

This is RViz eye-candy only: it publishes markers, never /tracked_obstacles, so it
does not feed the controller or perturb the predictive perception.
"""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def center_at(motion, cx, cy, ex, ey, radius, speed, t):
    """Analytic obstacle centre at time t; mirrors obstacle_field.hpp exactly."""
    if motion == 'circle':
        r = max(radius, 1e-9)
        ang = (speed / r) * t
        return cx + r * math.cos(ang), cy + r * math.sin(ang)
    if motion == 'line':
        dx, dy = ex - cx, ey - cy
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return cx, cy
        sx, sy = (cx, cy) if speed >= 0.0 else (ex, ey)
        tx, ty = (ex, ey) if speed >= 0.0 else (cx, cy)
        s = math.fmod(abs(speed) * t, 2.0 * length)
        frac = s / length if s <= length else (2.0 * length - s) / length
        return sx + frac * (tx - sx), sy + frac * (ty - sy)
    return cx, cy


class ObstacleMarkers(Node):
    """Publish the scenario obstacles as ground-truth cylinder markers."""

    def __init__(self):
        super().__init__('obstacle_markers')
        self.frame = self.declare_parameter('tracking_frame', 'odom').value
        odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.height = float(self.declare_parameter('body_height', 0.4).value)
        self.motion_eps = float(self.declare_parameter('motion_eps', 1.0e-3).value)

        # Empty-list defaults would be inferred as BYTE_ARRAY and reject the
        # STRING/DOUBLE arrays from the params file; declare them dynamically typed.
        dyn = ParameterDescriptor(dynamic_typing=True)

        def arr(name):
            return self.declare_parameter(name, None, dyn).value or []

        motion = arr('obs_motion')
        cx, cy = arr('obs_cx'), arr('obs_cy')
        ex, ey = arr('obs_ex'), arr('obs_ey')
        radius, speed, body = arr('obs_radius'), arr('obs_speed'), arr('obs_body')

        def get(seq, i, default):
            return float(seq[i]) if i < len(seq) else default

        self.obstacles = []
        for i, m in enumerate(motion):
            self.obstacles.append({
                'motion': m, 'cx': get(cx, i, 0.0), 'cy': get(cy, i, 0.0),
                'ex': get(ex, i, 0.0), 'ey': get(ey, i, 0.0),
                'radius': get(radius, i, 1.0), 'speed': get(speed, i, 0.0),
                'body': get(body, i, 0.25),
            })

        self.have_pose = False
        self.moved = False
        self.first_x = self.first_y = 0.0
        self.t0 = None
        self.stamp = None

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(MarkerArray, 'obstacle_bodies', qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.tick)
        self.get_logger().info(
            f'obstacle_markers: {len(self.obstacles)} body/bodies, frame={self.frame}')

    def on_odom(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.stamp = msg.header.stamp
        if not self.have_pose:
            self.have_pose = True
            self.first_x, self.first_y = px, py
        elif not self.moved and math.hypot(px - self.first_x, py - self.first_y) > self.motion_eps:
            self.moved = True
            self.t0 = rclpy.time.Time.from_msg(msg.header.stamp)

    def tick(self):
        if not self.have_pose or not self.obstacles:
            return
        t = 0.0
        if self.moved and self.t0 is not None:
            t = (rclpy.time.Time.from_msg(self.stamp) - self.t0).nanoseconds * 1e-9

        arr = MarkerArray()
        for i, o in enumerate(self.obstacles):
            x, y = center_at(o['motion'], o['cx'], o['cy'], o['ex'], o['ey'],
                             o['radius'], o['speed'], t)
            mk = Marker()
            mk.header.stamp = self.stamp
            mk.header.frame_id = self.frame
            mk.ns = 'obstacle_bodies'
            mk.id = i
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position = Point(x=x, y=y, z=self.height / 2.0)
            mk.pose.orientation.w = 1.0
            diameter = 2.0 * max(o['body'], 0.05)
            mk.scale.x = diameter
            mk.scale.y = diameter
            mk.scale.z = self.height
            mk.color = ColorRGBA(r=0.95, g=0.35, b=0.05, a=0.95)
            mk.lifetime = Duration(sec=0, nanosec=500_000_000)
            arr.markers.append(mk)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = ObstacleMarkers()
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
