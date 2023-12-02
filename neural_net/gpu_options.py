#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 15 17:23:14 2022
History:
04/15/2022: 

@author: jaerock
"""

import tensorflow as tf

def set():
    # to address the error:
    #   Could not create cudnn handle: CUDNN_STATUS_INTERNAL_ERROR
    
    # Allow GPU memory growth
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if gpus:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)