#!/usr/bin/env python3
"""
Trajectory Driver Node — smooth motion executor (Phase C).
Subscribes: /joint_command (IK goal angles)
Publishes:  /joint_states  (interpolated poses at 20 Hz)

Motion law: cubic smoothstep spline
    q(τ) = q0 + (q1 − q0)·(3τ² − 2τ³)
Zero velocity at both endpoints → physically smooth motion.

Duration law (time scaling):
    T = max joint travel / v_max   (floor: 1.0 s)
All joints arrive simultaneously, synchronized.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ['base_yaw', 'shoulder_pitch', 'elbow_pitch',
               'wrist_roll', 'wrist_pitch', 'tool_roll']
V_MAX = 1.0          # rad/s — matches URDF joint velocity limit
T_MIN = 1.0          # s — floor so tiny moves don't twitch
PERIOD = 0.05        # s — 20 Hz tick


class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')

        self.sub = self.create_subscription(
            JointState, 'joint_command', self.command_callback, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Live pose — evolves during motion, published every tick
        self.q = [0.0] * 6

        # Active trajectory (start, goal, timing)
        self.start = [0.0] * 6
        self.goal = [0.0] * 6
        self.elapsed = 0.0
        self.duration = T_MIN
        self.active = False

        self.timer = self.create_timer(PERIOD, self.tick)

        self.get_logger().info('Trajectory driver started — robot under spline control')

    def command_callback(self, msg):
        # Goal = current pose, updated only for joints named in the message
        goal = list(self.q)
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                goal[JOINT_NAMES.index(name)] = pos

        # Time law: scale duration to the largest joint travel
        travel = max(abs(g - c) for g, c in zip(goal, self.q))
        self.duration = max(travel / V_MAX, T_MIN)

        # KEY LINE: trajectory starts from wherever the arm is RIGHT NOW.
        # A new command mid-motion chains smoothly — no teleport, ever.
        self.start = list(self.q)
        self.goal = goal
        self.elapsed = 0.0
        self.active = True

        self.get_logger().info(
            f'New trajectory: max travel {math.degrees(travel):.1f}°, '
            f'duration {self.duration:.2f} s')

    def tick(self):
        if self.active:
            self.elapsed += PERIOD
            tau = min(self.elapsed / self.duration, 1.0)

            # ---- THE SPLINE: smoothstep 3τ² − 2τ³ ----
            s = tau * tau * (3.0 - 2.0 * tau)
            self.q = [a + (b - a) * s for a, b in zip(self.start, self.goal)]

            if tau >= 1.0:
                self.active = False
                self.q = list(self.goal)      # land exactly on target
                self.get_logger().info('Trajectory complete — arrived at target')

        # Heartbeat: publish live pose, idle or moving
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self.q)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()