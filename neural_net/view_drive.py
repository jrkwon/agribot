#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 10:07:31 2023

History:
11/14/2023: created for AGRIBOT

@author: jaerock
"""

from drive_view import DriveView

###############################################################################
#  for testing DriveView      
def main(weight_name, data_folder_name, target_folder_name):
    drive_view = DriveView(weight_name, data_folder_name, target_folder_name) 
    drive_view.run() # data folder path to test
       

###############################################################################
#       
if __name__ == '__main__':
    import sys
    try:
        if (len(sys.argv) == 3):
            main(None, sys.argv[1], sys.argv[2])
        elif (len(sys.argv) == 4):
            main(sys.argv[1], sys.argv[2], sys.argv[3])
        else:
            msg1 = 'Usage:\n$ python {}.py weight_name data_folder_name target_folder_name'.format(sys.argv[0]) 
            msg2 = '\n$ python {}.py data_folder_name target_folder_name'.format(sys.argv[0]) 
            msg = 'Use either of followings\n' + msg1 + msg2
            exit(msg)
        
    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
