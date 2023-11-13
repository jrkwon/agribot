#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:49:23 2017

History:
11/13/2023: start for AGRIBOT
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2
from vis.visualization import visualize_cam, overlay

from drive_run import DriveRun
from config import Config
import const
from image_process import ImageProcess


###############################################################################
#       
def main(model_path, image_file_path):
    image_process = ImageProcess()

    image = cv2.imread(image_file_path)

    image = image[Config.data_collection['image_crop_y1']:Config.data_collection['image_crop_y2'],
                    Config.data_collection['image_crop_x1']:Config.data_collection['image_crop_x2']]

    image = cv2.resize(image, 
                        (Config.neural_net['input_image_width'],
                         Config.neural_net['input_image_height']))
    image = image_process.process(image)

    drive_run = DriveRun(model_path)
    measurement = drive_run.run((image, ))

    plt.figure()
    plt.title('Steering Angle Prediction: ' + str(measurement[0][0]))
    heatmap = visualize_cam(drive_run.net_model.model, layer_name=const.LAST_CONV_LAYER, 
                filter_indices=None, seed_input=image) #, backprop_modifier='guided')
    heatmap = overlay(heatmap, image)

    plt.imshow(image)
    plt.imshow(heatmap) #, cmap='jet', alpha=0.5)

    # file name
    loc_slash = image_file_path.rfind('/')
    if loc_slash != -1: # there is '/' in the data path
        image_file_name = image_file_path[loc_slash+1:] 

    heatmap_file_path = model_path + '_' + image_file_name + '_cam.png'
    heatmap_file_path_pdf = model_path + '_' + image_file_name + '_cam.pdf'

    plt.tight_layout()
    # save fig    
    plt.savefig(heatmap_file_path, dpi=150)
    plt.savefig(heatmap_file_path_pdf, dpi=150)

    print('Saved ' + heatmap_file_path +' & .pdf')

    # show the plot 
    #plt.show()

###############################################################################
#       
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 3):
            exit('Usage:\n$ python {} model_path, image_file_name'.format(sys.argv[0]))

        main(sys.argv[1], sys.argv[2])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
