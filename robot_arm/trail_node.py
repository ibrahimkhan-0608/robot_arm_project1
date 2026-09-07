#!/usr/bin/env python3
"""
Trail Node — visualizes the tool-tip trajectory as a colored spline.
Subscribes:  /joint_states   (live robot poses, 20 Hz)
Publishes:   /visualization_marker  (LINE_STRIP marker → RViz trail)

Reuses the verified FK equations: joint angles → tool-tip (x, y, z).
Samples a trail point every 0.15 s while the arm is moving.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

SHOULDER_HEIGHT = 0.20
UPPER_ARM = 0.30
FOREARM = 0.30
TOOL = 0.15

TRAIL_SAMPLE_TIME = 0.15   # seconds between trail points
TRAIL_MAX_POINTS = 500     # trail memory limit


class TrailNode(Node):
    def __init__(self):
        super().__init__('trail_node')

        self.sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.last_position = None
        self.points = []
        self.last_sample_time = None

        self.get_logger().info('Trail node started — watching tool tip')

    def joint_callback(self, msg):
        q = dict(zip(msg.name, msg.position))

        theta1 = q.get('base_yaw', 0.0)
        theta2 = q.get('shoulder_pitch', 0.0)
        theta3 = q.get('elbow_pitch', 0.0)
        theta5 = q.get('wrist_pitch', 0.0)

        # ---- FK (verified equations from Phase A, extended to full 3D) ----
        x_planar = (UPPER_ARM * math.sin(theta2)
                    + FOREARM * math.sin(theta2 + theta3)
                    + TOOL * math.sin(theta2 + theta3 + theta5))
        z = (SHOULDER_HEIGHT
             + UPPER_ARM * math.cos(theta2)
             + FOREARM * math.cos(theta2 + theta3)
             + TOOL * math.cos(theta2 + theta3 + theta5))

        # Base rotation lifts the 2D plane into 3D (full XYZ FK)
        x = x_planar * math.cos(theta1)
        y = x_planar * math.sin(theta1)

        # ---- Sample only while the arm is MOVING ----
        now = self.get_clock().now()
        moved = (self.last_position is None
                 or abs(x - self.last_position[0]) > 1e-6
                 or abs(y - self.last_position[1]) > 1e-6
                 or abs(z - self.last_position[2]) > 1e-6)

        if moved:
            enough_time = (self.last_sample_time is None
                           or (now - self.last_sample_time).nanoseconds
                           / 1e9 > TRAIL_SAMPLE_TIME)
            if enough_time:
                self.points.append((x, y, z))
                self.last_sample_time = now
                if len(self.points) > TRAIL_MAX_POINTS:
                    self.points.pop(0)   # forget oldest — bounded memory
            self.last_position = (x, y, z)
            self.publish_trail()

    def publish_trail(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.ns = 'tool_trail'
        marker.id = 0
        marker.type = Marker.LINE_STRIP      # connected polyline = the spline
        marker.action = Marker.ADD
        marker.scale.x = 0.01                # 1 cm line width

        # Golden orange trail
        marker.color.r = 0.9
        marker.color.g = 0.5
        marker.color.b = 0.1
        marker.color.a = 1.0

        # ---- The completed marker-points block ----
        for px, py, pz in self.points:
            pt = Point()
            pt.x = px
            pt.y = py
            pt.z = pz
            marker.points.append(pt)

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrailNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()