#!/usr/bin/env python3
"""
Obstacle Node — renders the red pillar in RViz.
Publishes a translucent red CUBE marker at 1 Hz (republished so the
marker appears no matter when RViz connects — markers themselves
persist forever, but the initial message must be seen).
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from robot_arm.collision import X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacle)
        self.publish_obstacle()
        self.get_logger().info('Obstacle node started — red pillar active')

    def publish_obstacle(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns = 'obstacle'
        m.id = 1                              # separate from trail (id 0)
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Center of the box
        m.pose.position.x = (X_MIN + X_MAX) / 2.0
        m.pose.position.y = (Y_MIN + Y_MAX) / 2.0
        m.pose.position.z = (Z_MIN + Z_MAX) / 2.0
        m.pose.orientation.w = 1.0            # no rotation (axis-aligned)

        # Full dimensions
        m.scale.x = X_MAX - X_MIN             # 0.06
        m.scale.y = Y_MAX - Y_MIN             # 0.20
        m.scale.z = Z_MAX - Z_MIN             # 0.65

        # Translucent red — the arm stays visible if it ever enters
        m.color.r = 0.9
        m.color.g = 0.1
        m.color.b = 0.1
        m.color.a = 0.55

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()