#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017

History:
11/13/2023: modified for AGRIBOT
11/28/2020: modified for OSCAR 

@author: jaerock
"""
###############################################################################
# constant definition

#####################
# network model type
# CNN
NET_TYPE_JAEROCK     = 0
NET_TYPE_JAEROCK_VEL = 2
NET_TYPE_CE491       = 1
NET_TYPE_AGRIBOT     = 3
# LSTM
NET_TYPE_CONVLSTM    = 10

# file extension
DATA_EXT             = '.csv'
IMAGE_EXT            = '.jpg'
LOG_EXT              = '_log.csv'

# dir names for ckpt
CKPT_DIR             = 'ckpt'

######################
# data format
DATA_HEADER = "image_fname, steering_angle, throttle, brake, time, vel, vel_x, vel_y, vel_z, pos_x, pos_y, pos_z\n"
