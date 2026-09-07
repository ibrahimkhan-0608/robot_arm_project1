#!/usr/bin/env python3
"""
Forward Kinematics Node — 6-Axis Arm (2D planar mode)
Computes the tool tip position (x, z) from joint angles.
Phase A, Project 1: Robotic Arm Kinematics & Path Planning
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped


# ---- Arm geometry (from our URDF — single source of truth) ----
SHOULDER_HEIGHT = 0.20   # base (0.05) + column link1 (0.15)
UPPER_ARM = 0.30         # link2 length
FOREARM = 0.30           # link3 (0.25) + wrist1 link4 (0.05)
TOOL = 0.15              # link5 (0.05) + link6 tool (0.10)


class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')

        # Subscribe: listen to live joint angles
        self.sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Publish: computed tool tip position
        self.pub = self.create_publisher(PointStamped, 'tool_position', 10)

        self.get_logger().info('FK node started — waiting for joint states...')

    def joint_callback(self, msg):
        # Map joint names to angles (order is NOT guaranteed — always map by name)
        q = dict(zip(msg.name, msg.position))

        theta2 = q.get('shoulder_pitch', 0.0)
        theta3 = q.get('elbow_pitch', 0.0)
        theta5 = q.get('wrist_pitch', 0.0)

        # ---- THE FK EQUATIONS (from Step 7 theory) ----
        # Absolute segment tilts: angles stack along the chain
        a2 = theta2                          # upper arm tilt
        a3 = theta2 + theta3                 # forearm absolute tilt
        a5 = theta2 + theta3 + theta5        # tool segment absolute tilt

        x = (UPPER_ARM * math.sin(a2)
             + FOREARM * math.sin(a3)
             + TOOL * math.sin(a5))

        z = (SHOULDER_HEIGHT
             + UPPER_ARM * math.cos(a2)
             + FOREARM * math.cos(a3)
             + TOOL * math.cos(a5))

        # ---- Publish result as a ROS topic ----
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.point.x = x
        out.point.y = 0.0
        out.point.z = z
        self.pub.publish(out)

        self.get_logger().info(
            f'angles(θ2={math.degrees(theta2):.1f}°, θ3={math.degrees(theta3):.1f}°, '
            f'θ5={math.degrees(theta5):.1f}°) → tool(x={x:.3f}, z={z:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()