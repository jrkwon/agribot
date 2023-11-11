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
    #gpu_options.set()

    drive_train = DriveTrain(data_folder_name)
    drive_train.train(show_summary = True)

    
###############################################################################
#
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            exit('Usage:\n$ python {} data_path'.format(sys.argv[0]))

        train(sys.argv[1])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
