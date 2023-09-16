#! /usr/bin/env python3
import time  # Time library

import rclpy
from rclpy.node import Node, math
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2

from custom_msgs.msg import VisualNav
from geometry_msgs.msg import PoseStamped, Twist 
from sensor_msgs.msg import Image 
from builtin_interfaces.msg import Time


class NeuralNetNav(Node):
    
    def __init__(self):
        super().__init__('neural_net_nav')

        self.state = "wait" # wait, follow, stop
        self.command = VisualNav()
        self.horizontal_threshold = 325.0 # pixels
        self.timer_period = 0.08  #sec
        self.wakeup_limit = 5


        self.sleep_count = 0
        self.wakeup_count = 0

        self.get_logger().info("wait")

        # Horizontal FOV : 60 deg, Horizontal Res : 1280
        self.fov = 60
        self.h_res = 1280
        self.v_res = 720

        self.get_logger().info("create nav subscription")
        self.nav_subscription = self.create_subscription(
            VisualNav,
            '/visual_nav',
            self.nav_listener_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
                )

        self.timer = self.create_timer(
                self.timer_period,
                self.timer_stateful_action_callback 
                )

    def timer_stateful_action_callback(self):
        self.sleep_count += 1
        if self.sleep_count >= 0.5/self.timer_period:
            self.sleep_count = 0
            self.state = "wait"

        if self.state == "follow":
            self.get_logger().info("follow")
            self.follow_state()
        elif self.state == "wait":
            self.get_logger().info("wait")
            self.wait_state()
        
        elif self.state == "notfound":
            self.get_logger().info("notfound")



    def nav_listener_callback(self, msg : VisualNav):
        self.sleep_count = 0
        self.command = msg
        if self.command.operation == "follow":
            if self.state=="wait" and self.wakeup_count > self.wakeup_limit:
                self.wakeup_count = 0
                self.state="follow"

            elif self.state == "wait":
                self.wakeup_count += 1

            else:
                self.state ="follow"
        elif self.command.operation == "wait":
            self.wakeup_count = 0
            self.state = "wait"

        elif self.command.operation == "notfound":
            self.wakeup_count = 0
            if self.state == "wait":
                self.state = "wait"
            else:
                self.state = "notfound"

        else:
            self.get_logger().info("operation not support")


    def follow_state(self):
        horizontal_error = self.command.x - self.h_res/2
        twist_msg = Twist()

        # angular
        if horizontal_error > (self.h_res/2 - self.horizontal_threshold):
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.65 # right
        elif horizontal_error < -(self.h_res/2 -self.horizontal_threshold):
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.65 # left

        # linear + angular
        else:
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = - (horizontal_error / self.h_res) * 1.0

        
        self.get_logger().info(f"x: {self.command.x}")
        self.get_logger().info(f"linear: {twist_msg.linear.x} angular: {twist_msg.angular.z}")
        self.cmd_vel_publisher.publish(
                twist_msg
                )

        

    def wait_state(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(
                twist_msg
                )


        

def main(args=None):
    rclpy.init(args=args)

    neural_net_nav = NeuralNetNav()
    rclpy.spin(neural_net_nav)



    neural_net_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
