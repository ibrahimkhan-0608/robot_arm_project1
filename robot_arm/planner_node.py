#!/usr/bin/env python3
"""
Planner Node — the brain (Phase D).
Subscribes: /target_position (goal), /joint_states (pose feedback)
Publishes:  /joint_command (trajectory goals for the driver)

Plan: IK-solve goal → check direct spline vs obstacle → if blocked,
search candidate waypoints → execute multi-leg route with arrival
verification between legs.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

from robot_arm.collision import spline_collides

JOINT_NAMES = ['base_yaw', 'shoulder_pitch', 'elbow_pitch',
               'wrist_roll', 'wrist_pitch', 'tool_roll']

# IK geometry (same constants as ik_node)
SHOULDER_HEIGHT = 0.20
L1 = 0.30
L2 = 0.45

# Waypoint candidates: safe positions to route through when the direct
# path is blocked. Searched in order — first route with BOTH legs clear wins.
CANDIDATE_WAYPOINTS = [
    (0.30, 0.88),   # over the pillar, high
    (0.35, 0.85),   # over, further out
    (0.25, 0.90),   # over, higher
    (0.40, 0.80),   # over, far side
    (0.20, 0.88),   # over, left
    (0.15, 0.85),   # left, high
]

ARRIVAL_TOLERANCE = 0.03   # rad — joints this close = "arrived"
ARRIVAL_TIMEOUT = 30.0     # s — safety: never wait forever


def solve_ik(x, z):
    """2-link planar IK → joint vector, or None if unreachable."""
    dx = x
    dz = z - SHOULDER_HEIGHT
    r = math.sqrt(dx * dx + dz * dz)
    if r > L1 + L2 or r < abs(L1 - L2):
        return None
    cos_beta = (L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    theta3 = math.pi - beta
    psi = math.atan2(dx, dz)
    alpha = math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3))
    theta2 = psi - alpha
    return [0.0, theta2, theta3, 0.0, 0.0, 0.0]


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.sub = self.create_subscription(
            PointStamped, 'target_position', self.target_callback, 10)
        self.js_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(JointState, 'joint_command', 10)

        self.current_q = [0.0] * 6
        self.pending_goal = None    # final goal waiting after waypoint leg
        self.wait_started = 0.0

        self.get_logger().info(
            'Planner node started — collision-aware routing active')

    # ---------- pose feedback ----------
    def joint_callback(self, msg):
        q = dict(zip(msg.name, msg.position))
        self.current_q = [q.get(n, 0.0) for n in JOINT_NAMES]

        if self.pending_goal is not None:
            if self.arrived(self.pending_goal):
                goal = self.pending_goal
                self.pending_goal = None
                self.get_logger().info(
                    'Waypoint reached — commanding final goal')
                self.send_command(goal)
            elif self.timed_out():
                goal = self.pending_goal
                self.pending_goal = None
                self.get_logger().warn(
                    'Arrival timeout — sending final goal anyway')
                self.send_command(goal)

    def arrived(self, goal):
        return all(abs(c - g) < ARRIVAL_TOLERANCE
                   for c, g in zip(self.current_q, goal))

    def timed_out(self):
        return (time.time() - self.wait_started) > ARRIVAL_TIMEOUT

    # ---------- planning ----------
    def target_callback(self, msg):
        if self.pending_goal is not None:
            self.get_logger().warn('Busy executing a plan — target ignored')
            return

        x, z = msg.point.x, msg.point.z
        q_goal = solve_ik(x, z)
        if q_goal is None:
            self.get_logger().error(
                f'Target ({x:.2f}, {z:.2f}) unreachable — outside workspace')
            return

        # ---- 1. try the direct route ----
        if not spline_collides(self.current_q, q_goal):
            self.get_logger().info(
                f'Direct path to ({x:.2f}, {z:.2f}) is clear — executing')
            self.send_command(q_goal)
            return

        # ---- 2. blocked → search safe waypoints ----
        self.get_logger().warn(
            f'Direct path to ({x:.2f}, {z:.2f}) COLLIDES with the obstacle '
            '— searching safe waypoints')
        for wx, wz in CANDIDATE_WAYPOINTS:
            q_wp = solve_ik(wx, wz)
            if q_wp is None:
                continue                      # waypoint itself unreachable
            if spline_collides(self.current_q, q_wp):
                continue                      # leg 1 blocked
            if spline_collides(q_wp, q_goal):
                continue                      # leg 2 blocked
            # ---- safe route found ----
            self.get_logger().info(
                f'Routing via waypoint ({wx:.2f}, {wz:.2f}) — both legs clear')
            self.pending_goal = q_goal
            self.wait_started = time.time()
            self.send_command(q_wp)           # leg 1 now
            return                            # leg 2 sent on arrival

        self.get_logger().error(
            'No safe route found — all waypoints blocked. Target rejected.')

    # ---------- output ----------
    def send_command(self, q_goal):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(q_goal)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()