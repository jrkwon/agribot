#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import threading 
import cv2
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import math

import sys
import os

import const
from image_converter import ImageConverter
from config import Config
from image_process import ImageProcess
#import gpu_options
from drive_run import DriveRun

if Config.data_collection['vehicle_name'] == 'scout':
    from scout_msgs.msg import ScoutControl
else:
    exit(Config.data_collection['vehicle_name'] + 'not supported vehicle.')


dc_config = Config.data_collection
nn_config = Config.neural_net
rn_config = Config.run_neural
velocity = 0


class NeuralControl:
    def __init__(self, weight_file_name, weight_file_name2 = None):
        try:
            rospy.init_node('run_neural', log_level=rospy.ERROR)
            # ... rest of your code ...

            # -------------------------------------------------------------------
            # teleop_twist_joy scaling
            # default values from teleop_logitech.yaml were used.
            prefix = dc_config['teleop_twist_node_prefix']
            self.scale_linear = rospy.get_param(prefix + 'scale_linear', 0.4)
            self.scale_angular = rospy.get_param(prefix + 'scale_angular', 0.6)

            self.image_processed = False

            self.ic = ImageConverter()
            self.image_process = ImageProcess()
            self.rate = rospy.Rate(30)

            self.drive= DriveRun(weight_file_name)
            if weight_file_name2 != None:
                self.drive2 = DriveRun(weight_file_name2) # multiple network models can be used
            else:
                self.drive2 = None
            rospy.Subscriber(dc_config['camera_image_topic'], Image, self._controller_cb)
            self.image = None
            #self.config = Config()
            self.braking = False

        except Exception as e:
            print("Exception during initialization: ", e)

    def _controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        cropped = img[nn_config['image_crop_y1']:nn_config['image_crop_y2'],
                      nn_config['image_crop_x1']:nn_config['image_crop_x2']]
                      
        img = cv2.resize(cropped, (nn_config['input_image_width'],
                                   nn_config['input_image_height']))
                                  
        self.image = self.image_process.process(img)

        ## this is for CNN-LSTM net models
        if nn_config['lstm'] is True:
            self.image = np.array(self.image).reshape(1, 
                                 nn_config['input_image_height'],
                                 nn_config['input_image_width'],
                                 nn_config['input_image_depth'])
        self.image_processed = True
        
    def _timer_cb(self):
        self.braking = False

    def apply_brake(self):
        self.braking = True
        timer = threading.Timer(rn_config['brake_apply_sec'], self._timer_cb) 
        timer.start()

      
def pos_vel_cb(value):
    global velocity

    vel_x = value.twist.twist.linear.x 
    vel_y = value.twist.twist.linear.y
    vel_z = value.twist.twist.linear.z
    
    velocity = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

        
def main(weight_file_name, weight_file_name2 = None):
    #gpu_options.set()

    # ready for neural network
    neural_control = NeuralControl(weight_file_name, weight_file_name2)
    
    rospy.Subscriber(dc_config['base_pose_topic'], Odometry, pos_vel_cb)
    # ready for topic publisher
    joy_pub = rospy.Publisher(dc_config['vehicle_control_topic'], ScoutControl, queue_size = 10)
    joy_data = ScoutControl()
    joy_data.gearshift = ScoutControl.FORWARD

    #if Config.data_collection['vehicle_name'] == 'rover':
    #    joy_pub4mavros = rospy.Publisher(Config.config['mavros_cmd_vel_topic'], Twist, queue_size=20)

    print('\n------------------------------------------------')
    print('Start running. Vroom. Vroom. Vroooooom......')
    print('------------------------------------------------')
    print('steering \tthrottle: \tbrake \tvelocity')

    use_predicted_throttle = True if nn_config['num_outputs'] == 2 else False
    while not rospy.is_shutdown():

        if neural_control.image_processed is False:
            continue
        
        # predicted steering angle from an input image
        if nn_config['num_inputs'] == 2:
            prediction = neural_control.drive.run((neural_control.image, velocity))
            if weight_file_name2 != None:
                prediction2 = neural_control.drive2.run((neural_control.image, velocity))
            if nn_config['num_outputs'] == 2:
                # prediction is [ [] ] numpy.ndarray
                joy_data.steering = prediction[0][0]*neural_control.scale_angular
                joy_data.throttle = prediction[0][1]*neural_control.scale_linear
            else: # num_outputs is 1
                joy_data.steering = prediction[0][0]*neural_control.scale_angular
        else: # num_inputs is 1
            prediction = neural_control.drive.run((neural_control.image, ))
            if weight_file_name2 != None:
                prediction2 = neural_control.drive2.run((neural_control.image, ))
            if nn_config['num_outputs'] == 2:
                # prediction is [ [] ] numpy.ndarray
                joy_data.steering = prediction[0][0]*neural_control.scale_angular
                joy_data.throttle = prediction[0][1]*neural_control.scale_linear
            else: # num_outputs is 1
                joy_data.steering = prediction[0][0]*neural_control.scale_angular
            
        #############################
        ## very very simple controller
        ## 

        is_sharp_turn = False
        # if brake is not already applied and sharp turn
        if neural_control.braking is False: 
            if velocity < rn_config['velocity_0']: # too slow then no braking
                joy_data.throttle = rn_config['throttle_default']*neural_control.scale_linear 
                joy_data.brake = 0
            elif abs(joy_data.steering) > rn_config['sharp_turn_min']:
                is_sharp_turn = True
            
            if is_sharp_turn or velocity > rn_config['max_vel']: 
                joy_data.throttle = rn_config['throttle_sharp_turn']*neural_control.scale_linear
                joy_data.brake = rn_config['brake_val']*neural_control.scale_linear
                neural_control.apply_brake()
            else:
                if use_predicted_throttle is False:
                    joy_data.throttle = rn_config['throttle_default']*neural_control.scale_linear
                joy_data.brake = 0
        
        joy_pub.publish(joy_data)

        ## print out
        cur_output = '{0:.3f} \t\t{1:.3f} \t\t{2:.3f} \t{3:.3f}\r'.format(joy_data.steering, 
                          joy_data.throttle, joy_data.brake, velocity)

        sys.stdout.write(cur_output)
        sys.stdout.flush()
        
        ## ready for processing a new input image
        neural_control.image_processed = False
        neural_control.rate.sleep()



if __name__ == "__main__":
    try:
        if len(sys.argv) == 2:
            main(sys.argv[1])
        elif len(sys.argv) == 3:
            main(sys.argv[1], sys.argv[2])
        else:
            exit('Usage:\n$ rosrun run_neural run_neural.py weight_file_name [weight_file_name2]')


    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
        
