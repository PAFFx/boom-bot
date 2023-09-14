#! /usr/bin/env python3
import time  # Time library

import rclpy
from rclpy.node import Node, math
from rclpy.duration import Duration


from custom_msgs.msg import VisualNav
from geometry_msgs.msg import PoseStamped 

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, Spin

class NeuralNetNav(Node):
    
    def __init__(self):
        super().__init__('neural_net_nav')

        self.navigator = BasicNavigator()
        self.state = "wait" # wait, follow, stop

        # TODO: set_inital pose
        # self.navigator.setInitialPose(init_pose)
        
        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        self.navigator.waitUntilNav2Active()

        # Horizontal FOV : 60 deg, Horizontal Res : 1280
        self.fov = 60
        self.h_res = 1280
        self.frame_id = "base_link"
        self.last_goal_msg = PoseStamped()

        self.subscription = self.create_subscription(
            VisualNav,
            '/visual_nav',
            self.listener_callback,
            10)

    def listener_callback(self, msg : VisualNav):
        op = msg.operation
        if op == "follow":
            self.state="follow"
            self.follow_state(msg)
        elif op == "wait":
            self.state = "wait"
            self.wait_state()

        elif op == "notfound" and self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.state = "spin"
            self.navigator.spin()

        else:
            print("Invalid operation command")






    def follow_state(self, msg):
        x_pos,y_pos = self.calc_robot_position_goal(msg.horizontal_center_offset, msg.distance)
        # Publish goal
        self.last_goal_msg.header.stamp = self.navigator.get_clock().now()
        self.last_goal_msg.header.frame_id = self.frame_id
        self.last_goal_msg.pose.position.x = x_pos
        self.last_goal_msg.pose.position.y = y_pos
        #TODO: Add orientation

        self.navigator.goToPose([self.last_goal_msg])

    def calc_robot_position_goal(self, x_offset,distance):
        deg_offset = ( x_offset * self.fov)/ (self.h_res)
        rad_offset = (deg_offset * math.pi) / 180

        x_pos = distance * math.cos(rad_offset)
        y_pos = distance * math.sin(rad_offset)

        return x_pos, y_pos

    def wait_state(self):
        self.navigator.cancelTask()
    
        

def main(args=None):
    rclpy.init(args=args)

    neural_net_nav = NeuralNetNav()
    rclpy.spin(neural_net_nav)



    neural_net_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
