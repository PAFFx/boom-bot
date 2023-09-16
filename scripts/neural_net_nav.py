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


def construct_depth_image(raw, width, height):
    arr = np.array(raw, dtype=np.uint8).reshape(-1, 2)
    arr = arr[:, 0] + (arr[:, 1] * 2**8)
    arr = arr.reshape(width, height)
    return arr

class NeuralNetNav(Node):
    
    def __init__(self):
        super().__init__('neural_net_nav')

        self.state = "wait" # wait, follow, stop
        self.command = VisualNav()
        self.distance = 0.0
        self.distance_threshold = 1000.0 # millimetres
        self.horizontal_threshold = 50.0 # pixels

        self.get_logger().info("wait")

        # Horizontal FOV : 60 deg, Horizontal Res : 1280
        self.fov = 60
        self.h_res = 1280
        self.v_res = 720

        # Create subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        print("create nav subscription")
        self.nav_subscription = self.create_subscription(
            VisualNav,
            '/visual_nav',
            self.nav_listener_callback,
            10)
        print("create depth subscription")
        self.depth_subscription = self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self.depth_listener_callback,
                qos_profile
                )

        self.cmd_vel_publisher = self.create_publisher(
                Twist,
                '/cmd_vel',
                10
                )

        self.timer = self.create_timer(
                0.01,
                self.timer_stateful_action_callback 
                )

    def timer_stateful_action_callback(self):
        if self.state == "follow":
            print("follow")
            self.follow_state()
        elif self.state == "wait":
            print("wait")
            self.wait_state()
        
        elif self.state == "notfound":
            print("notfound")


    def depth_listener_callback(self, msg : Image):
        self.depth_data = construct_depth_image(msg.data, msg.width, msg.height)

    def nav_listener_callback(self, msg : VisualNav):
        self.command = msg
        if self.command.operation == "follow":
            self.state="follow"
        elif self.command.operation == "wait":
            self.state = "wait"

        elif self.command.operation == "notfound":
            self.state = "notfound"

        else:
            print("Invalid operation command")


    def follow_state(self):
        # read depth
        x_depth_pos = int((self.command.x / self.h_res) * 640)
        y_depth_pos = int((self.command.y / self.v_res) * 480)
        horizontal_error = self.command.x - self.h_res/2
        depth = 0
        try: 
            depth = self.depth_data[x_depth_pos, y_depth_pos]
        except:
            print("Error reading depth data")
        self.distance = depth

        twist_msg = Twist()
        # linear + angular
        if self.distance > self.distance_threshold and horizontal_error > self.horizontal_threshold or horizontal_error < -self.horizontal_threshold :
            twist_msg.linear.x = 0.5
            twist_msg.angular.z = - (horizontal_error / self.h_res) * 1.0
        # linear
        elif self.distance > self.distance_threshold:
            twist_msg.linear.x = 1.0
        # angular
        elif horizontal_error > self.horizontal_threshold or horizontal_error < -self.horizontal_threshold:
            twist_msg.angular.z = -(horizontal_error / self.h_res ) * 1.0

        
        print("x:",twist_msg.linear.x)
        print("z:",twist_msg.angular.z)
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
