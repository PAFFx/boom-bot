#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import GoalOffset

class GoalOffsetNav(Node):

  def __init__(self):
    super().__init__('goal_offset_nav')
    self.subscription = self.create_subscription(
      GoalOffset,
      'goal_offset',
      self.listener_callback,
      10)
    self.subscription

  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
  rclpy.init(args=args)
  goal_offset_nav = GoalOffsetNav()
  rclpy.spin(goal_offset_nav)
  goal_offset_nav.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()