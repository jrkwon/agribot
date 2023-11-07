#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:49:23 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""


import sys
from drive_train import DriveTrain
import gpu_options
import tensorflow as tf


###############################################################################
#
def train(data_folder_name):
    '''
    gpu_options.set()

    drive_train = DriveTrain(data_folder_name)
    drive_train.train(show_summary = False)
    '''

    gpus = tf.config.list_physical_devices('GPU')
    if gpus:
    # Restrict TensorFlow to only allocate 1GB of memory on the first GPU
        try:
            tf.config.set_logical_device_configuration(
                gpus[0],
                [tf.config.LogicalDeviceConfiguration(memory_limit=1024)])
            logical_gpus = tf.config.list_logical_devices('GPU')
            print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
        except RuntimeError as e:
            # Virtual devices must be set before GPUs have been initialized
            print(e)

            
    drive_train = DriveTrain(data_folder_name)
    drive_train.train(show_summary = False)


###############################################################################
#
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            exit('Usage:\n$ python {} data_path'.format(sys.argv[0]))

        train(sys.argv[1])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
