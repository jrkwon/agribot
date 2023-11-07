#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 12:23:14 2019
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""


import sys
import os
from progressbar import ProgressBar

import const
from drive_data import DriveData
from config import Config
import pandas as pd
###############################################################################
#
def build_csv(data_path):
    # add '/' at the end of data_path if user doesn't specify
    if data_path[-1] != '/':
        data_path = data_path + '/'

    print("data_path",data_path)

    # find the second '/' from the end to get the folder name
    loc_dir_delim = data_path[:-1].rfind('/')
    if (loc_dir_delim != -1):
        folder_name = data_path[loc_dir_delim+1:-1]
        csv_file = folder_name + const.DATA_EXT
    else:
        folder_name = data_path[:-1]
        csv_file = folder_name + const.DATA_EXT

    csv_file = data_path + folder_name +const.DATA_EXT
    print("csv_file",csv_file)
    print("folder_name",folder_name)
    '''
    csv_backup_name = data_path + csv_file + '.bak'
    os.rename(data_path + csv_file, csv_backup_name)
    print('rename ' + data_path + csv_file + ' to ' + csv_backup_name)
    '''
    #data = DriveData(csv_backup_name)
    #data.read(normalize = False)
    data = pd.read_csv(csv_file, index_col=False)
    Steering_ang = list(map(float,data.loc[1:,'steering_angle'].to_list()))
    throttle_n=list(map(float,data.loc[1:,'throttle'].to_list()))
    brake_n=list(map(float,data.loc[1:,'brake'].to_list()))
    linux_time_n   = list(map(float,data.loc[1:,'time'].to_list()))
    vel_n          = list(map(float,data.loc[1:,'vel'].to_list()))
    vel_x_n        = list(map(float,data.loc[1:,'vel_x'].to_list()))
    vel_y_n        = list(map(float,data.loc[1:,'vel_y'].to_list()))
    vel_z_n        = list(map(float,data.loc[1:,'vel_z'].to_list()))
    pos_x_n        = list(map(float,data.loc[1:,'pos_x'].to_list()))
    pos_y_n        = list(map(float,data.loc[1:,'pos_y'].to_list()))
    pos_z_n        = list(map(float,data.loc[1:,'pos_z'].to_list()))
    image_fname_n  = data.loc[1:,'image_fname'].to_list()

    new_csv = []
    
    # check image exists
    bar = ProgressBar()
    print('len(data)',len(data))
    for i in bar(range(1,len(data))):
        print("i" , i)
        if os.path.exists(data_path + image_fname_n[i]):
            if Config.data_collection['brake'] is True:
                new_csv.append(image_fname_n[i] + ','
                            + str(Steering_ang[i]) + ','
                            + str(throttle_n[i]) + ','
                            + str(brake_n[i]) + ',' # brake
                            + str(linux_time_n[i]) + ','
                            + str(vel_n[i]) + ','
                            + str(vel_x_n[i]) + ','
                            + str(vel_y_n[i]) + ','
                            + str(vel_z_n[i]) + ','
                            + str(pos_x_n[i]) + ','
                            + str(pos_y_n[i]) + ','
                            + str(pos_z_n[i]) + '\n')
            else:
                new_csv.append(image_fname_n[i] + ','
                            + str(Steering_ang[i]) + ','
                            + str(throttle_n[i]) + ','
                            + str(linux_time_n[i]) + ','
                            + str(vel_n[i]) + ','
                            + str(vel_x_n[i]) + ','
                            + str(vel_y_n[i]) + ','
                            + str(vel_z_n[i]) + ','
                            + str(pos_x_n[i]) + ','
                            + str(pos_y_n[i]) + ','
                            + str(pos_z_n[i]) + '\n')

    # write a new csv
    new_csv_fh = open(data_path + csv_file, 'w')
    for i in range(len(new_csv)):
        new_csv_fh.write(new_csv[i])
    new_csv_fh.close()


###############################################################################
#
def main():
    if (len(sys.argv) != 2):
        print('Usage: \n$ python rebuild_csv data_folder_name')
        return

    build_csv(sys.argv[1])


###############################################################################
#
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nShutdown requested. Exiting...')
