#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import rospy
import cv2
import os
import numpy as np
import datetime
import time
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
#from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import math

import image_converter as ic
import const
from config import Config
import config 
from sensor_msgs.msg import Joy

config = Config.data_collection
if config['vehicle_name'] == 'scout':
    from scout_msgs.msg import ScoutControl
else:
    exit(config['vehicle_name'] + ' not supported vehicle.')


class DataCollection():
    def __init__(self):
        self.steering = 0
        self.throttle = 0
        self.brake = 0

        self.vel_x = self.vel_y = self.vel_z = 0
        self.vel = 0
        self.pos_x = self.pos_y = self.pos_z = 0

        self.img_cvt = ic.ImageConverter()

        ##
        # data will be saved in a location specified with rosparam path_to_e2e_data

        # create csv data file
        name_datatime = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
        #path = '../data/' + sys.argv[1] + '/' + name_datatime + '/'
        path = rospy.get_param('path_to_e2e_data', 
                        './e2e-dataset') + '/' + sys.argv[1] + '/' + name_datatime + '/'
        if os.path.exists(path):
            print('The path exists. continuing...')
        else:
            print('A new folder created: ' + path)
            os.makedirs(path)

        self.text = open(str(path) + name_datatime + const.DATA_EXT, "w+")
        #line = "image_fname, steering_angle, throttle, brake, time, vel, vel_x, vel_y, vel_z, pos_x, pos_y, pos_z\n"
        self.text.write(const.DATA_HEADER)

        self.path = path


    def calc_velocity(self, x, y, z):
        return math.sqrt(x**2 + y**2 + z**2)


    def steering_throttle_cb(self, value):
        self.throttle = value.throttle
        self.steering = value.steering
        self.brake = value.brake


    def pos_vel_cb(self, value):
        self.pos_x = value.pose.pose.position.x 
        self.pos_y = value.pose.pose.position.y
        self.pos_z = value.pose.pose.position.z

        self.vel_x = value.twist.twist.linear.x 
        #self.vel_y = value.twist.twist.linear.y
        self.vel_y = value.twist.twist.angular.z
        self.vel_z = value.twist.twist.linear.z
        self.vel = self.calc_velocity(self.vel_x, self.vel_y, self.vel_z)


    def recorder_cb(self, data):
        img = self.img_cvt.imgmsg_to_opencv(data)

        unix_time = time.time()
        time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
        file_full_path = str(self.path) + str(time_stamp) + const.IMAGE_EXT

        cv2.imwrite(file_full_path, img)
        sys.stdout.write(file_full_path + '\r')

        line = "{}{},{:.4f},{:.4f},{:.4f},{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\r\n".format(
                                                time_stamp, const.IMAGE_EXT, 
                                                self.steering, 
                                                self.throttle,
                                                self.brake,
                                                unix_time,
                                                self.vel,
                                                self.vel_x,
                                                self.vel_y,
                                                self.vel_z,
                                                self.pos_x,
                                                self.pos_y,
                                                self.pos_z)

        self.text.write(line)                                                 


def main():
    dc = DataCollection()

    print(config['vehicle_control_topic'])
    print(config['base_pose_topic'])
    print(config['camera_image_topic'])

    rospy.init_node('data_collection')
    #rospy.Subscriber(config['vehicle_control_topic'], Joy, dc.steering_throttle_cb)
    rospy.Subscriber(config['vehicle_control_topic'], ScoutControl, dc.steering_throttle_cb)
    rospy.Subscriber(config['base_pose_topic'], Odometry, dc.pos_vel_cb)
    rospy.Subscriber(config['camera_image_topic'], Image, dc.recorder_cb)

    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nBye...")    


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: ')

        
        exit('$ rosrun data_collection data_collection.py your_data_id')

    main()