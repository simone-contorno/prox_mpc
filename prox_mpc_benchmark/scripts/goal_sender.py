#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Auto-send a scenario's goal(s) to Nav2, replacing the manual action call.

A single goal uses NavigateToPose; a waypoint set uses NavigateThroughPoses
(modes a / b2). Exits 0 on SUCCEEDED, non-zero otherwise, so the orchestrator can
record the action result.
"""

import argparse
import math
import sys

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def pose(x, y, yaw, frame, clock):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = clock.now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    qz, qw = yaw_to_quat(float(yaw))
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class GoalSender(Node):
    def __init__(self):
        super().__init__('prox_mpc_goal_sender')


def parse_points(spec):
    pts = []
    for triple in spec.split(';'):
        triple = triple.strip()
        if not triple:
            continue
        x, y, yaw = (float(v) for v in triple.split(','))
        pts.append((x, y, yaw))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--points', required=True,
                    help="semicolon-separated x,y,yaw goals e.g. '3,0,0' or '1,0,0;3,0,0'")
    ap.add_argument('--frame', default='map')
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = GoalSender()
    pts = parse_points(args.points)

    if len(pts) == 1:
        client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        goal = NavigateToPose.Goal()
        goal.pose = pose(*pts[0], args.frame, node.get_clock())
    else:
        client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose(*p, args.frame, node.get_clock()) for p in pts]

    if not client.wait_for_server(timeout_sec=30.0):
        node.get_logger().error('Nav2 action server unavailable')
        rclpy.shutdown()
        return 2

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('goal rejected')
        rclpy.shutdown()
        return 3

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
    if not result_future.done():
        node.get_logger().error('goal timed out')
        rclpy.shutdown()
        return 4

    status = result_future.result().status
    SUCCEEDED = 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
    node.get_logger().info(f'goal finished with status {status}')
    rclpy.shutdown()
    return 0 if status == SUCCEEDED else 5


if __name__ == '__main__':
    sys.exit(main())
