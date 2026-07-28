#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Ground-truth dynamic-obstacle publisher - the ORACLE for the feasibility gate.

Publishes /tracked_obstacles (prox_mpc_msgs/ObstacleArray, odom frame) with the
EXACT obstacle state and future trajectory from the analytic scenario motion law
(mirrors obstacle_field.hpp obstacleCenterAt). It is a drop-in replacement for the
IMM tracker that gives the predictive controller perfect perception + perfect
prediction. Two uses:

  * Feasibility gate: if proxmpc_pred fed this oracle still collides on a cell,
    the cell is infeasible for this robot at this speed cap (discard it) - not a
    controller failure. If the oracle solves it, the cell is provably solvable.
  * Perception ablation: oracle-pred vs tracker-pred on the same solvable cell
    isolates whether a failure is the tracker (perception) or the controller.

Obstacle params match the scan-simulator arrays (written by run_nav2). Only moving
obstacles (circle/line) are published; static structure is left to the costmap.
The obstacle clock is anchored to the robot's first motion, identical to
scan_simulator / metrics_node, so the phase matches the driven trajectory.
"""

import math

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from prox_mpc_msgs.msg import Obstacle, ObstacleArray
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


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


class GtObstaclePublisher(Node):
    """Publish ground-truth moving obstacles as /tracked_obstacles."""

    def __init__(self):
        super().__init__('gt_obstacle_publisher')
        self.tracking_frame = self.declare_parameter('tracking_frame', 'odom').value
        odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        # Match the controller horizon (np * dt) so the perfect prediction lines
        # up node-for-node with the solver's evaluation points.
        self.horizon_n = int(self.declare_parameter('horizon_n', 20).value)
        self.pred_dt = float(self.declare_parameter('pred_dt', 0.1).value)
        self.motion_eps = float(self.declare_parameter('motion_eps', 1.0e-3).value)

        # Empty-list defaults would be inferred as BYTE_ARRAY and reject the
        # STRING/DOUBLE arrays from the params file; declare them dynamically typed.
        dyn = ParameterDescriptor(dynamic_typing=True)

        def arr(name):
            return self.declare_parameter(name, None, dyn).value or []

        motion = arr('obs_motion')
        cx = arr('obs_cx')
        cy = arr('obs_cy')
        ex = arr('obs_ex')
        ey = arr('obs_ey')
        radius = arr('obs_radius')
        speed = arr('obs_speed')
        body = arr('obs_body')

        def get(seq, i, default):
            return float(seq[i]) if i < len(seq) else default

        self.obstacles = []
        for i, m in enumerate(motion):
            if m in ('circle', 'line'):
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
        self.pub = self.create_publisher(ObstacleArray, 'tracked_obstacles', qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.tick)
        self.get_logger().info(
            f'gt_obstacle_publisher: {len(self.obstacles)} moving obstacle(s), '
            f'frame={self.tracking_frame}, horizon={self.horizon_n}x{self.pred_dt}s')

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
        if not self.have_pose:
            return
        t = 0.0
        if self.moved and self.t0 is not None:
            t = (rclpy.time.Time.from_msg(self.stamp) - self.t0).nanoseconds * 1e-9

        arr = ObstacleArray()
        arr.header.stamp = self.stamp
        arr.header.frame_id = self.tracking_frame
        h = 1.0e-3
        for i, o in enumerate(self.obstacles):
            x, y = center_at(o['motion'], o['cx'], o['cy'], o['ex'], o['ey'],
                             o['radius'], o['speed'], t)
            xf, yf = center_at(o['motion'], o['cx'], o['cy'], o['ex'], o['ey'],
                               o['radius'], o['speed'], t + h)
            xb, yb = center_at(o['motion'], o['cx'], o['cy'], o['ex'], o['ey'],
                               o['radius'], o['speed'], t - h)
            ob = Obstacle()
            ob.id = i
            ob.position = Point(x=x, y=y, z=0.0)
            ob.velocity = Vector3(x=(xf - xb) / (2.0 * h), y=(yf - yb) / (2.0 * h), z=0.0)
            ob.radius = o['body']
            ob.position_covariance = [0.0, 0.0, 0.0, 0.0]
            ob.velocity_covariance = [0.0, 0.0, 0.0, 0.0]  # perfect: no uncertainty growth
            pred = []
            for k in range(self.horizon_n):
                pxk, pyk = center_at(o['motion'], o['cx'], o['cy'], o['ex'], o['ey'],
                                     o['radius'], o['speed'], t + (k + 1) * self.pred_dt)
                pred.append(Point(x=pxk, y=pyk, z=0.0))
            ob.predicted_positions = pred
            ob.prediction_dt = self.pred_dt
            arr.obstacles.append(ob)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = GtObstaclePublisher()
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
