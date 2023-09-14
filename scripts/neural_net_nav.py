#! /usr/bin/env python3

import rclpy
from rclpy.node import Node, math
from rclpy.duration import Duration

import time  # Time library

from custom_msgs.msg import VisualNav
from geometry_msgs.msg import PoseStamped 


class NeuralNetNav(Node):

    def __init__(self):
        super().__init__('neural_net_nav')

        # Horizontal FOV : 60 deg, Horizontal Res : 1280
        self.fov = 60
        self.h_res = 1280

        self.subscription = self.create_subscription(
            VisualNav,
            '/visual_nav',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        angular_goal = self.calc_robot_angular_goal(msg.horizontal_center_offset)
        self.get_logger().info(f'Angular Goal : {angular_goal}')

    def calc_robot_angular_goal(self, x_offset):
        deg_offset = ( x_offset * self.fov)/ (self.h_res)
        rad_offset = (deg_offset * math.pi) / 180

        return rad_offset
        
         
        


def main(args=None):
    rclpy.init(args=args)

    neural_net_nav = NeuralNetNav()

    rclpy.spin(neural_net_nav)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    neural_net_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
