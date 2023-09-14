#! /usr/bin/env python3
import time  # Time library

import rclpy
from rclpy.node import Node, math
from rclpy.duration import Duration


from custom_msgs.msg import VisualNav
from geometry_msgs.msg import PoseStamped 
from builtin_interfaces.msg import  Time


class NeuralNetNav(Node):

    def __init__(self):
        super().__init__('neural_net_nav')

        # Horizontal FOV : 60 deg, Horizontal Res : 1280
        self.fov = 60
        self.h_res = 1280
        self.frame_id = "base_link"

        self.subscription = self.create_subscription(
            VisualNav,
            '/visual_nav',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        self.publisher # prevent unused variable warning


    def listener_callback(self, msg):
        x_pos,y_pos = self.calc_robot_position_goal(msg.horizontal_center_offset, msg.distance)
        self.get_logger().info(f'X,Y Goal : {x_pos}, {y_pos}')
            
        time_obj = Time()
        time_obj.sec = self.get_clock().now().seconds_nanoseconds()[0]

        # publish goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = time_obj
        goal_msg.header.frame_id = self.frame_id
        goal_msg.pose.position.x = x_pos
        goal_msg.pose.position.y = y_pos

    def calc_robot_position_goal(self, x_offset,distance):
        deg_offset = ( x_offset * self.fov)/ (self.h_res)
        rad_offset = (deg_offset * math.pi) / 180

        x_pos = distance * math.cos(rad_offset)
        y_pos = distance * math.sin(rad_offset)

        return x_pos, y_pos
        


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
