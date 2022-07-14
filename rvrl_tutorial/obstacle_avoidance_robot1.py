#!/usr/bin/env python

# author: Arnoud Visser  
# date: July 13, 2022
# license: free to use for educational and research purposes

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range    

import random
from math import pi
from statistics import mean

def rad2deg(x): return x*180/pi

class ObstacleAvoidance(Node):

    def __init__(self):

        super().__init__('obstacle_avoidance')

        self.so2 = self.create_subscription(Range, '/robot1/so2/range', self.so2_callback, 10)
        self.so3 = self.create_subscription(Range, '/robot1/so3/range', self.so3_callback, 10)
        self.so4 = self.create_subscription(Range, '/robot1/so4/range', self.so4_callback, 10)
        self.so5 = self.create_subscription(Range, '/robot1/so5/range', self.so5_callback, 10)

        self.pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        timer_period = 3.0  # 0.33 Hz  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.COR_DIST = 0.75
        self.obstacle_front_right = False
        self.obstacle_FFR = False
        self.obstacle_FFL = False
        self.obstacle_front_left = False

    def so2_callback(self, msg):

        if (msg.range) < self.COR_DIST:
            print('obstacle in front-left')
            print('distances %f' % msg.range)
            self.obstacle_front_left = True
        else:
            self.obstacle_front_left = False

    def so3_callback(self, msg):

        if (msg.range) < self.COR_DIST:
            print('obstacle in front-front-left')
            print('distances %f' % msg.range)
            self.obstacle_FFL = True
        else:
            self.obstacle_FFL = False

    def so4_callback(self, msg):

        if (msg.range) < self.COR_DIST:
            print('obstacle in front-front-right')
            print('distances %f' % msg.range)
            self.obstacle_FFR = True
        else:
            self.obstacle_FFR = False

    def so5_callback(self, msg):

        if (msg.range) < self.COR_DIST:
            print('obstacle in front-right')
            print('distances %f' % msg.range)
            self.obstacle_front_right = True
        else:
            self.obstacle_front_right = False

    def timer_callback(self):
        out_msg = Twist()
        out_msg.linear.x = random.random() * 0.2
        out_msg.angular.z = random.triangular(-1.0,1.0,0.0) * 0.4

        if(self.obstacle_front_left == True or self.obstacle_front_right == True ):
            out_msg.linear.x = -random.random() * 0.05
            out_msg.angular.z = random.triangular(-1.0,1.0,0.0) * 0.1

        if(self.obstacle_FFR == True):
            out_msg.angular.z = random.triangular(0.0,1.0,0.0) * 0.4

        if(self.obstacle_FFL == True):
            out_msg.angular.z = random.triangular(-1.0,0.0,0.0) * 0.4

        print('timer callback #%d with speed %f and angular speed %f' % (self.i,out_msg.linear.x, out_msg.angular.z))

        self.pub.publish(out_msg)
        self.i += 1
    


def main(args=None):

    rclpy.init(args=args)

    move = ObstacleAvoidance()
    rclpy.spin(move)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

