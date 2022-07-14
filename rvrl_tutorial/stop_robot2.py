#!/usr/bin/env python

# author: Arnoud Visser 
# date: July 13, 2022
# license: free to use for educational and research purposes

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import random

class StopMove(Node):

    def __init__(self):

        super().__init__('stop_move')

        self.pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        timer_period = 1.0  # 1 Hz  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        print('timer callback #%d' % self.i)
        out_msg = Twist()
        out_msg.linear.x = 0.0
        out_msg.angular.z = 0.0
        self.pub.publish(out_msg)
        self.i += 1
    


def main(args=None):

    rclpy.init(args=args)

    stop_move = StopMove()
    rclpy.spin_once(stop_move)
    stop_move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

