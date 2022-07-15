#!/usr/bin/env python

# author: Arnoud Visser  
# date: July 15, 2022
# license: free to use for educational and research purposes

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image    

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge
import cv2 

import random
from math import pi
from statistics import mean

def rad2deg(x): return x*180/pi

class ObstacleAvoidanceDepth(Node):

    def __init__(self):

        super().__init__('obstacle_avoidance_depth')

        self.depth = self.create_subscription(Image, '/robot1/camera/depth/image_raw', self.depth_callback, 10)

        self.pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        timer_period = 1.0  # 0.33 Hz  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.COR_DIST = 0.5
        self.obstacle_front_right = 0
        self.obstacle_FFR = 0
        self.obstacle_front = 0
        self.obstacle_FFL = 0
        self.obstacle_front_left = 0
        self.obstacle_distance = 5.0 # out of range

        self.bridge = CvBridge()

    def depth_callback(self, msg):

        # Image coordinates of the center pixel
        u = msg.width / 2
        v = msg.height / 2

        # Image coordinates of the center pixel in the lower part
        fl = int(msg.width * 0.1)
        ffl = int(msg.width * 0.3)
        f = int(msg.width * 0.5)
        fr = int(msg.width * 0.7)
        ffr = int(msg.width * 0.9)
        v = int(msg.height * 0.8)

        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1") # 4 chars one float

        depth_fl  = cv_image[v, fl] - 0.1  # camera is further back
        depth_ffl = cv_image[v, ffl] - 0.1
        depth_f   = cv_image[v, f] - 0.1 
        depth_ffr = cv_image[v, ffr] - 0.1
        depth_fr  = cv_image[v, fr] - 0.1 

        # Output the measure
        # print('Center distance : value %f pixel for pixel (%d,%d) with sonar %f m)' % (depth_f, v, f, self.obstacle_distance))

        # to counter noise: count

        if (depth_fl < self.COR_DIST):
            self.obstacle_front_left += 1
        else:
            self.obstacle_front_left -= 1

        if (depth_ffl < self.COR_DIST):
            self.obstacle_FFL += 1  
        else:
            self.obstacle_FFL -= 1

        if (depth_ffl < self.COR_DIST):
            self.obstacle_front += 1  
        else:
            self.obstacle_front -= 1

        if (depth_ffr < self.COR_DIST):
            self.obstacle_FFR += 1
        else:
            self.obstacle_FFR -= 1

        if (depth_fr < self.COR_DIST):
            self.obstacle_front_right += 1  
        else:
            self.obstacle_front_right -= 1


    def timer_callback(self):

        out_msg = Twist()

        out_msg.angular.z = random.triangular(-1.0,1.0,0.0) * 0.4

        if(self.i > 1):
            out_msg.linear.x = random.random() * 0.2
        else:
            out_msg.linear.x  =  0.0 # don't rush forward at the first timestep
            out_msg.angular.z = -1.57 # start with looking around


        if(self.obstacle_front_right > 0):
            print('obstacle in front-right')
            out_msg.linear.x = out_msg.linear.x / 2.0
            out_msg.angular.z = random.triangular(0.0,1.0,0.0) * 0.4
        else:
           self.obstacle_front_right = 0 # don't let the count go negative

        if(self.obstacle_front_left > 0):
            print('obstacle in front-left')
            out_msg.linear.x = out_msg.linear.x / 2.0
            out_msg.angular.z = random.triangular(-1.0,0.0,0.0) * 0.4
        else:
            self.obstacle_front_left = 0

        if(self.obstacle_FFL > 0):
            print('obstacle in front-front-left')
            out_msg.linear.x = -random.random() * 0.05
            out_msg.angular.z = random.triangular(-0.0,1.0,0.0) * 0.1
        else:
            self.obstacle_FFL = 0
  
        if(self.obstacle_FFR > 0):
            print('obstacle in front-front-right')
            out_msg.linear.x = -random.random() * 0.05
            out_msg.angular.z = random.triangular(-1.0,0.0,0.0) * 0.1
        else:
            self.obstacle_FFR = 0

        if(self.obstacle_front > 0):
            print('obstacle in front')
            out_msg.linear.x = -random.random() * 0.1
            out_msg.angular.z = 0.0
        else:
            self.obstacle_front = 0

        print('timer callback #%d with speed %f and angular speed %f' % (self.i,out_msg.linear.x, out_msg.angular.z))

        self.pub.publish(out_msg)
        self.i += 1
    


def main(args=None):

    rclpy.init(args=args)

    move = ObstacleAvoidanceDepth()
    rclpy.spin(move)
    move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

