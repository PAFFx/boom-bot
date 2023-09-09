#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

ser = serial.Serial("/dev/ttyUSB0", 56700)

pwm_min = -255
pwm_max = 255

rads_min = -4
rads_max = 4

def linear_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min)  + out_min

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        left_wheel_rps = msg.velocity[0]
        right_wheel_rps = msg.velocity[1]

        self.get_logger().info(f"Received: left: {left_wheel_rps} rad/s, right: {right_wheel_rps} rad/s\n")

        left_wheel_pwm = int(linear_map(left_wheel_rps, rads_min, rads_max, pwm_min, pwm_max))
        right_wheel_pwm = int(linear_map(right_wheel_rps, rads_min, rads_max, pwm_min, pwm_max))

        self.get_logger().info(f"Sending: left: {left_wheel_pwm} pwm, right: {right_wheel_pwm} pwm\n")
        
        try:
            ser.write(f"{left_wheel_pwm},{right_wheel_pwm}".encode())
        except:
            return


def main(args=None):
    rclpy.init(args=args)
    subscriber = MotorDriver()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()