#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""


import pandas as pd
from progressbar import ProgressBar
import matplotlib.pyplot as plt
import numpy as np
import random

from config import Config

class DriveData:

    if Config.data_collection['brake'] is True:
        csv_header = ['image_fname', 'steering_angle', 'throttle', 'brake', 
                    'linux_time', 
                    'vel', 'vel_x', 'vel_y', 'vel_z',
                    'pos_x', 'pos_y', 'pos_z' ]
    else:
        csv_header = ['image_fname', 'steering_angle', 'throttle', 
                    'linux_time', 
                    'vel', 'vel_x', 'vel_y', 'vel_z',
                    'pos_x', 'pos_y', 'pos_z' ]

    def __init__(self, csv_fname):
        self.csv_fname = csv_fname
        self.df = None
        self.image_names = []
        self.measurements = []
        self.time_stamps = []
        self.velocities = []
        self.velocities_xyz = []
        self.positions_xyz = []
        self.Steering_ang=[]
        self.throttle_n=[]
        self.brake_n=[]
        self.linux_time_n = []
        self.vel_n=[]
        self.vel_x_n=[]
        self.vel_y_n=[]
        self.vel_z_n=[]
        self.pos_x_n=[]
        self.pos_y_n=[]
        self.pos_z_n=[]
        self.image_fname_n=[]

    def read(self, read = True, show_statistics = True, normalize = True):
        self.df = pd.read_csv(self.csv_fname, names=self.csv_header, index_col=False)
        #self.fname = fname
        self.linux_time_n   = list(map(float,self.df.loc[1:,'linux_time'].to_list()))
        self.vel_n          = list(map(float,self.df.loc[1:,'vel'].to_list()))
        self.vel_x_n        = list(map(float,self.df.loc[1:,'vel_x'].to_list()))
        self.vel_y_n        = list(map(float,self.df.loc[1:,'vel_y'].to_list()))
        self.vel_z_n        = list(map(float,self.df.loc[1:,'vel_z'].to_list()))
        self.pos_x_n        = list(map(float,self.df.loc[1:,'pos_x'].to_list()))
        self.pos_y_n        = list(map(float,self.df.loc[1:,'pos_y'].to_list()))
        self.pos_z_n        = list(map(float,self.df.loc[1:,'pos_z'].to_list()))
        #self.image_fname_n  = self.df.loc[1:,'image_fname']
        self.image_fname_n  = self.df.loc[1:,'image_fname'].to_list()
        self.Steering_ang = list(map(float,self.df.loc[1:,'steering_angle'].to_list()))
        self.throttle_n=list(map(float,self.df.loc[1:,'throttle'].to_list()))
        self.brake_n=list(map(float,self.df.loc[1:,'brake'].to_list()))
        print(len(self.image_fname_n))

        

        ############################################
        # show statistics
        if (show_statistics):
            print('\n####### data statistics #########')
            print('Steering Command Statistics:')
            print(self.df['steering_angle'].describe())

            print('\nThrottle Command Statistics:')
            # Throttle Command Statistics
            print(self.df['throttle'].describe())

            if Config.data_collection['brake'] is True:
                print('\nBrake Command Statistics:')
                # Throttle Command Statistics
                print(self.df['brake'].describe())
                
            if Config.neural_net['num_outputs'] == 2:
                print('\nVelocity Command Statistics:')
                # Throttle Command Statistics
                print(self.df['vel'].describe())

        ############################################
        # normalize data
        # 'normalize' arg is for overriding 'normalize_data' config.
        if (Config.neural_net['normalize_data'] and normalize):
            print('\nnormalizing... wait for a moment')
            num_bins = 50
            fig, (ax1, ax2) = plt.subplots(1, 2)
            #fig.suptitle('Data Normalization')
            
            self.Steering_ang = list(map(float,self.df.loc[1:,'steering_angle'].to_list()))
            self.throttle_n=list(map(float,self.df.loc[1:,'throttle'].to_list()))
            self.brake_n=list(map(float,self.df.loc[1:,'brake'].to_list()))
            '''
            print('self.Steering_ang',self.Steering_ang)
            print(self.df.loc[ 1: ,'steering_angle'  ].shape)
            '''
            hist, bins = np.histogram(self.Steering_ang, bins=num_bins)
            center = (bins[:-1] + bins[1:])*0.5
            ax1.bar(center, hist, width=0.05)
            ax1.set(title = 'original')

            remove_list = []
            samples_per_bin = 200



            for j in range(num_bins):
                list_ = []
                for i in range(1,len(self.df['steering_angle'])-1):
                    if float(self.df.loc[i,'steering_angle']) >= bins[j] and float(self.df.loc[i,'steering_angle']) <= bins[j+1]:
                        list_.append(i)
                random.shuffle(list_)
                list_ = list_[samples_per_bin:]
                remove_list.extend(list_)
            
            print('\r####### data normalization #########')
            print('removed:', len(remove_list))
            self.df.drop(self.df.index[remove_list], inplace = True)
            self.df.reset_index(inplace = True)
            self.df.drop(['index'], axis = 1, inplace = True)
            print('remaining:', len(self.df))
            
            hist, _ = np.histogram(self.Steering_ang, (num_bins))
            ax2.bar(center, hist, width=0.05)
            ax2.plot((np.min(self.Steering_ang), np.max(self.Steering_ang)), 
                        (samples_per_bin, samples_per_bin))  
            ax2.set(title = 'normalized')          

            plt.tight_layout()
            plt.savefig(self.get_data_path() + '_normalized.png', dpi=150)
            plt.savefig(self.get_data_path() + '_normalized.pdf', dpi=150)
            #plt.show()

        ############################################ 
        # read out
        if (read): 
            num_data = len(self.df)
            print("num_data",num_data-1)
            bar = ProgressBar()
            
            for i in bar(range(1,num_data-1)): # we don't have a title
                #print("self.image_fname_n[i]",self.image_fname_n[i],i)
                self.image_names.append(self.image_fname_n[i])
                #print(len(self.image_names))
                if Config.data_collection['brake'] is True:
                    self.measurements.append((float(self.Steering_ang[i]),
                                            float(self.throttle_n[i]), 
                                            float(self.brake_n[i])))
                else:
                    self.measurements.append((float(self.Steering_ang[i]),
                                            float(self.throttle_n[i]), 
                                            0.0)) # dummy value for old data
                self.time_stamps.append(float(self.linux_time_n[i]))
                self.velocities.append(float(self.vel_n[i]))
                self.velocities_xyz.append((float(self.vel_x_n[i]), 
                                            float(self.vel_y_n[i]), 
                                            float(self.vel_z_n[i])))
                self.positions_xyz.append((float(self.pos_x_n[i]), 
                                            float(self.pos_y_n[i]), 
                                            float(self.pos_z_n[i])))


    def get_data_path(self):
        loc_slash = self.csv_fname.rfind('/')
        
        if loc_slash != -1: # there is '/' in the data path
            data_path = self.csv_fname[:loc_slash] # get folder name
            return data_path
        else:
            exit('ERROR: csv file path must have a separator.')


###############################################################################
#  for testing DriveData class only
def main(data_path):
    import const

    if data_path[-1] == '/':
        data_path = data_path[:-1]

    loc_slash = data_path.rfind('/')
    if loc_slash != -1: # there is '/' in the data path
        model_name = data_path[loc_slash + 1:] # get folder name
        #model_name = model_name.strip('/')
    else:
        model_name = data_path
    csv_path = data_path + '/' + model_name + const.DATA_EXT   
    
    data = DriveData(csv_path)
    data.read(read = False)


###############################################################################
#       
if __name__ == '__main__':
    import sys

    try:
        if (len(sys.argv) != 2):
            exit('Usage:\n$ python {} data_path'.format(sys.argv[0]))

        main(sys.argv[1])

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
