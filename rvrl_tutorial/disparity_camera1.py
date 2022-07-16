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

import numpy as np

def rad2deg(x): return x*180/pi

class ObstacleAvoidanceDepth(Node):

    def __init__(self):

        super().__init__('obstacle_avoidance_depth')

        self.depth = self.create_subscription(Image, '/robot1/camera/depth/image_raw', self.depth_callback, 10)

        self.twist = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.disparity = self.create_publisher(Image, '/robot1/camera/depth/disparity_image', 10)

        timer_period = 1.0  # 0.33 Hz  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.COR_DIST = 0.5
        self.detection_height = 0.70 # relative to top of image
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
        v = int(msg.height * self.detection_height)

        #cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1") # 4 chars one float
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        # depth -0.1 because camera is further back

        depth_fl  = np.mean(cv_image[v-10:v+10, fl-10:fl+10]) - 0.1
        depth_ffl = np.mean(cv_image[v-10:v+10, ffl-10:ffl+10]) - 0.1
        depth_f   = np.mean(cv_image[v-10:v+10, f-10:f+10]) - 0.1
        depth_ffr   = np.mean(cv_image[v-10:v+10, ffr-10:ffr+10]) - 0.1
        depth_fr   = np.mean(cv_image[v-10:v+10, fr-10:fr+10]) - 0.1

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

        if (depth_f < self.COR_DIST):
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

        # officialy calculated with disparity = B*f / depth
        # B is the baseline, the distance between the cams, 
        # and f the focal length

        # camera is realsense r200, so baseline is 70mm
        # see https://github.com/IntelRealSense/librealsense/blob/v1.12.1/doc/camera_specs.md 
        # the focal length f_x and f_y are intrinsic parameters, which depends on the  depending on the resolution/aspect ratio of the requested images
        # see https://github.com/IntelRealSense/librealsense/blob/v1.12.1/doc/projection.md#intrinsic-camera-parameters
        
        # calibration from https://www.researchgate.net/profile/Kailun-Yang/publication/320874005/figure/tbl1/AS:684764398899204@1540271963614/Calibration-parameters-of-Intel-RealSense-R200.png
        # From article Yang, Kailun et al. ‘IR Stereo RealSense: Decreasing Minimum Range of Navigational Assistance for Visually Impaired Individuals’. 1 Jan. 2017 : 743 – 755, DOI: 10.3233/AIS-170459

        # baseline 69.95 mm, IR focal length ~(580,580) pixels, RGB focal length ~ (1400,1400) pixels 
        
        focal_length = 580
        hack_baseline = 0.07
        cv_image = (hack_baseline * focal_length) / cv_image   

        grey_image = cv2.addWeighted(cv_image, 1, 0 ,0 ,0, dtype=cv2.CV_8U)
        color_image = cv2.applyColorMap(grey_image, cv2.COLORMAP_JET)

        if(self.obstacle_front_left > 0):
            color_image[v-10:v+10, fl-10:fl+10] = (0,0,255)
        else:
            color_image[v-10:v+10, fl-10:fl+10] = (0,255,0)

        if(self.obstacle_FFL > 0):
            color_image[v-10:v+10, ffl-10:ffl+10] = (0,0,255)
        else:
            color_image[v-10:v+10, ffl-10:ffl+10] = (0,255,0)

        if(self.obstacle_front > 0):
            color_image[v-10:v+10, f-10:f+10] = (0,0,255)
        else:
            color_image[v-10:v+10, f-10:f+10] = (0,255,0)

        if(self.obstacle_FFR > 0):
            color_image[v-10:v+10, ffr-10:ffr+10] = (0,0,255)
        else:
            color_image[v-10:v+10, ffr-10:ffr+10] = (0,255,0)

        if(self.obstacle_front_right > 0):
            color_image[v-10:v+10, fr-10:fr+10] = (0,0,255)
        else:
            color_image[v-10:v+10, fr-10:fr+10] = (0,255,0)

#        cv2.imwrite('/tmp/DisparityImage.png', color_image)

        out_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="passthrough") 
        self.disparity.publish(out_msg)

    def timer_callback(self):

        out_msg = Twist()

        out_msg.angular.z = random.triangular(-1.0,1.0,0.0) * 0.4
        out_msg.linear.x = random.random() * 0.2

#        self.twist.publish(out_msg)
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

