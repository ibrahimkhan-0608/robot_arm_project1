#!/usr/bin/env python3
"""
Inverse Kinematics Node — 6-Axis Arm (2-link planar solution)
Subscribes: target tool position (x, z)  →  Publishes: joint angles
The heart of Project 1: position in, motor angles out.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState


# ---- Geometry constants (must match URDF — same as fk_node) ----
SHOULDER_HEIGHT = 0.20
L1 = 0.30                 # upper arm
L2 = 0.45                 # forearm + tool (locked straight, wrist θ5 = 0)


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # Subscribe: incoming target position (we type it via command line)
        self.sub = self.create_subscription(
            PointStamped, 'target_position', self.target_callback, 10)

        # Publish: solved joint angles → makes the robot move
        self.pub = self.create_publisher(
            JointState, 'joint_command', 10)

        self.get_logger().info(
            'IK node started — publish a target on /target_position')

    def target_callback(self, msg):
        x = msg.point.x
        z = msg.point.z

        # ---------- STEP 1: target relative to SHOULDER ----------
        # (all angles are measured from the shoulder, not the ground)
        dx = x
        dz = z - SHOULDER_HEIGHT

        # ---------- STEP 2: reachability check ----------
        r = math.sqrt(dx * dx + dz * dz)

        if r > L1 + L2 or r < abs(L1 - L2):
            self.get_logger().error(
                f'Target ({x:.2f}, {z:.2f}) unreachable: '
                f'r={r:.3f} outside [{abs(L1-L2):.2f}, {L1+L2:.2f}] m')
            return                                   # report, don't crash

        # ---------- STEP 3: Law of Cosines → elbow ----------
        # β = interior angle at the elbow
        cos_beta = (L1*L1 + L2*L2 - r*r) / (2 * L1 * L2)
        cos_beta = max(-1.0, min(1.0, cos_beta))    # guard: float rounding
        beta = math.acos(cos_beta)

        theta3 = math.pi - beta                     # θ3 = deviation from straight

        # ---------- STEP 4: shoulder angle ----------
        psi = math.atan2(dx, dz)                    # direction shoulder→target
        alpha = math.atan2(L2 * math.sin(theta3),   # aim correction
                           L1 + L2 * math.cos(theta3))
        theta2 = psi - alpha

        # ---------- STEP 5: verify by running FK on the answer ----------
        x_check = L1 * math.sin(theta2) + L2 * math.sin(theta2 + theta3)
        z_check = (SHOULDER_HEIGHT + L1 * math.cos(theta2)
                   + L2 * math.cos(theta2 + theta3))
        err = math.sqrt((x - x_check)**2 + (z - z_check)**2)

        # ---------- STEP 6: publish joint angles ----------
        msg_out = JointState()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.name = ['base_yaw', 'shoulder_pitch', 'elbow_pitch',
                        'wrist_roll', 'wrist_pitch', 'tool_roll']
        msg_out.position = [0.0, theta2, theta3, 0.0, 0.0, 0.0]
        self.pub.publish(msg_out)

        self.get_logger().info(
            f'Target ({x:.2f}, {z:.2f}) → θ2={math.degrees(theta2):.1f}° '
            f'θ3={math.degrees(theta3):.1f}° | FK check → '
            f'({x_check:.3f}, {z_check:.3f}) | error={err:.5f} m')

        if err > 0.001:
            self.get_logger().warn('Verification error > 1 mm — investigate!')


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()