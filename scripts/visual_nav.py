#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import VisualNav
from geometry_msgs.msg import Twist

wheel_max_speed = 1.848  # rad/s


class VisualNavNode(Node):
    def __init__(self):
        super().__init__("visual_nav_node")
        self.subscription = self.create_subscription(
            VisualNav, "visual_nav", self.listener_callback, 10
        )
        self.subscription
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.publisher_

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    visual_nav_node = VisualNavNode()

    rclpy.spin(visual_nav_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visual_nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
