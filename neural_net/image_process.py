#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import cv2
import numpy as np

class ImageProcess:

    def process(self, img):
        return self._normalize(img)


    def _equalize_histogram(self, img):
        # img is expected as RGB

        img_yuv = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2RGB)
        return img
    

    def _normalize(self, img):

        img_norm = np.zeros_like(img)

        cv2.normalize(img, img_norm, 0, 255, cv2.NORM_MINMAX)
        return img_norm
