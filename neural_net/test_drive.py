#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from drive_log import DriveLog
    

###############################################################################
#       
def main(weight_name, data_folder_name):
    
    drive_log = DriveLog(weight_name, data_folder_name) 
    drive_log.run() # data folder path to test
       

###############################################################################
#       
if __name__ == '__main__':
    import sys

    try:
        if (len(sys.argv) != 3):
            exit('Usage:\n$ python {} weight_name data_folder_name'.format(sys.argv[0]))
        
        main(sys.argv[1], sys.argv[2])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')

