#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017

History:
11/13/2023: modified for AGRIBOT
11/28/2020: modified for OSCAR 

@author: jaerock
"""

from keras.models import Sequential, Model
from keras.layers import Lambda, Dropout, Flatten, Dense, Activation, concatenate
from keras.layers import Conv2D, Convolution2D, MaxPooling2D, BatchNormalization, Input
from keras import losses, optimizers
import tensorflow as tf

import const
from config import Config
import utilities


config = Config.neural_net
config_rn = Config.run_neural


def model_agribot():
    input_shape = (config['input_image_height'],
                    config['input_image_width'],
                    config['input_image_depth'])

    return Sequential([
        Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
        Conv2D(filters=32, kernel_size=3, activation='relu'),
        MaxPooling2D(pool_size=2),
        Conv2D(filters=64, kernel_size=3, activation='relu'),
        MaxPooling2D(pool_size=2),
        Conv2D(filters=128, kernel_size=3, activation='relu'),
        MaxPooling2D(pool_size=2),
        #Conv2D(filters=256, kernel_size=3, activation='relu'),
        #MaxPooling2D(pool_size=2),
        Conv2D(filters=256, kernel_size=3, activation='relu', name='conv2d_last'),
        MaxPooling2D(pool_size=2),
        Flatten(),
        Dense(1024),
        #Dropout(0.5),
        Dense(128),
        #Dropout(0.5),
        Dense(12),
        #Dropout(0.5),
        Dense(config['num_outputs'])])


def model_ce491():
    input_shape = (config['input_image_height'],
                    config['input_image_width'],
                    config['input_image_depth'])

    return Sequential([
        Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
        Conv2D(24, (5, 5), strides=(2,2), activation='relu'),
        Conv2D(36, (5, 5), strides=(2,2), activation='relu'),
        Conv2D(48, (5, 5), strides=(2,2), activation='relu'),
        Conv2D(64, (3, 3), activation='relu'),
        Conv2D(64, (3, 3), activation='relu', name='conv2d_last'),
        Flatten(),
        Dense(100, activation='relu'),
        Dense(50, activation='relu'),
        Dense(10, activation='relu'),
        Dense(config['num_outputs'])])

def model_jaerock():
    input_shape = (config['input_image_height'],
                    config['input_image_width'],
                    config['input_image_depth'])

    return Sequential([
        Lambda(lambda x: x/127.5 - 1.0, input_shape=input_shape),
        Conv2D(24, (5, 5), strides=(2,2)),
        Conv2D(36, (5, 5), strides=(2,2)),
        Conv2D(48, (5, 5), strides=(2,2)),
        Conv2D(64, (3, 3)),
        Conv2D(64, (3, 3), name='conv2d_last'),
        Flatten(),
        Dense(1000),
        Dense(100),
        Dense(50),
        Dense(10),
        Dense(config['num_outputs'])])
    
def model_jaerock_vel():
    img_shape = (config['input_image_height'],
                    config['input_image_width'],
                    config['input_image_depth'],)
    vel_shape = 1
    ######img model#######
    img_input = Input(shape=img_shape)
    lamb = Lambda(lambda x: x/127.5 - 1.0)(img_input)
    conv_1 = Conv2D(24, (5, 5), strides=(2,2))(lamb)
    conv_2 = Conv2D(36, (5, 5), strides=(2,2))(conv_1)
    conv_3 = Conv2D(48, (5, 5), strides=(2,2))(conv_2)
    conv_4 = Conv2D(64, (3, 3))(conv_3)
    conv_5 = Conv2D(64, (3, 3), name='conv2d_last')(conv_4)
    flat = Flatten()(conv_5)
    fc_1 = Dense(1000, name='fc_1')(flat)
    fc_2 = Dense(100, name='fc_2')(fc_1)
    
    ######vel model#######
    vel_input = Input(shape=[vel_shape])
    fc_vel = Dense(50, name='fc_vel')(vel_input)
    
    ######concat##########
    concat_img_vel = concatenate([fc_2, fc_vel])
    fc_3 = Dense(50, name='fc_3')(concat_img_vel)
    fc_4 = Dense(10, name='fc_4')(fc_3)
    fc_last = Dense(2, name='fc_str')(fc_4)
    
    model = Model(inputs=[img_input, vel_input], output=fc_last)

    return model

def model_convlstm():
    from keras.layers import LSTM
    from keras.layers import TimeDistributed

    # redefine input_shape to add one more dims
    img_shape = (None, config['input_image_height'],
                    config['input_image_width'],
                    config['input_image_depth'])
    vel_shape = (None, 1)
    
    input_img = Input(shape=img_shape, name='input_image')
    lamb      = TimeDistributed(Lambda(lambda x: x/127.5 - 1.0), name='lamb_img')(input_img)
    conv_1    = TimeDistributed(Convolution2D(24, (5, 5), strides=(2,2)), name='conv_1')(lamb)
    conv_2    = TimeDistributed(Convolution2D(36, (5, 5), strides=(2,2)), name='conv_2')(conv_1)
    conv_3    = TimeDistributed(Convolution2D(48, (5, 5), strides=(2,2)), name='conv_3')(conv_2)
    conv_4    = TimeDistributed(Convolution2D(64, (3, 3)), name='conv_4')(conv_3)
    conv_5    = TimeDistributed(Convolution2D(64, (3, 3)), name='conv2d_last')(conv_4)
    flat      = TimeDistributed(Flatten(), name='flat')(conv_5)
    fc_1      = TimeDistributed(Dense(1000, activation='relu'), name='fc_1')(flat)
    fc_2      = TimeDistributed(Dense(100, activation='relu' ), name='fc_2')(fc_1)
    
    if config['num_inputs'] == 1:
        lstm      = LSTM(10, return_sequences=False, name='lstm')(fc_2)
        fc_3      = Dense(50, activation='relu', name='fc_3')(lstm)
        fc_4      = Dense(10, activation='relu', name='fc_4')(fc_3)
        fc_last   = Dense(config['num_outputs'], activation='linear', name='fc_last')(fc_4)
    
        model = Model(inputs=input_img, outputs=fc_last)
        
    elif config['num_inputs'] == 2:
        input_velocity = Input(shape=vel_shape, name='input_velocity')
        lamb      = TimeDistributed(Lambda(lambda x: x / 38), name='lamb_vel')(input_velocity)
        fc_vel_1  = TimeDistributed(Dense(50, activation='relu'), name='fc_vel')(lamb)
        concat    = concatenate([fc_2, fc_vel_1], name='concat')
        lstm      = LSTM(10, return_sequences=False, name='lstm')(concat)
        fc_3      = Dense(50, activation='relu', name='fc_3')(lstm)
        fc_4      = Dense(10, activation='relu', name='fc_4')(fc_3)
        fc_last   = Dense(config['num_outputs'], activation='linear', name='fc_last')(fc_4)

        model = Model(inputs=[input_img, input_velocity], outputs=fc_last)
    
    return model

class NetModel:
    def __init__(self, model_path):
        self.model = None
        model_name = model_path[model_path.rfind('/'):] # get folder name
        self.name = model_name.strip('/')

        self.model_path = model_path
        #self.config = Config()

        ### --> move to gpu_options to support using multiple network models
        ## to address the error:
        ##   Could not create cudnn handle: CUDNN_STATUS_INTERNAL_ERROR

        self._model()

    ###########################################################################
    #
    def _model(self):
        if config['network_type'] == const.NET_TYPE_JAEROCK:
            self.model = model_jaerock()
        elif config['network_type'] == const.NET_TYPE_AGRIBOT:
            self.model = model_agribot()
        elif config['network_type'] == const.NET_TYPE_JAEROCK_VEL:
            self.model = model_jaerock_vel()
        elif config['network_type'] == const.NET_TYPE_CE491:
            self.model = model_ce491()
        elif config['network_type'] == const.NET_TYPE_CONVLSTM:
            self.model = model_convlstm()
        else:
            exit('ERROR: Invalid neural network type.')

        self.summary()
        self._compile()



    # ###########################################################################
    # #
    # def _mean_squared_error(self, y_true, y_pred):
    #     diff = K.abs(y_true - y_pred)
    #     if (diff < config['steering_angle_tolerance']) is True:
    #         diff = 0
    #     return K.mean(K.square(diff))

    ###########################################################################
    #
    def _compile(self):
        if config['lstm'] is True:
            learning_rate = config['lstm_lr']
        else:
            learning_rate = config['cnn_lr']

        self.model.compile(loss=losses.mean_squared_error,
        #            optimizer=optimizers.RMSprop(learning_rate=learning_rate), 
                    optimizer=optimizers.Adam(learning_rate=learning_rate), 
                    metrics=['accuracy'])
        #            metrics=['mae'])
        # if config['steering_angle_tolerance'] == 0.0:
        #     self.model.compile(loss=losses.mean_squared_error,
        #               optimizer=optimizers.Adam(),
        #               metrics=['accuracy'])
        # else:
        #     self.model.compile(loss=losses.mean_squared_error,
        #               optimizer=optimizers.Adam(),
        #               metrics=['accuracy', self._mean_squared_error])


    ###########################################################################
    #
    # save model
    def save(self, model_name):

        self.model.save(model_name + '.keras')
        """
        json_string = self.model.to_json()
        #weight_filename = self.model_path + '_' + Config.config_yaml_name \
        #    + '_N' + str(config['network_type'])
        open(model_name+'.json', 'w').write(json_string)
        self.model.save_weights(model_name+'.h5', overwrite=True)
        """


    ###########################################################################
    # model_path = '../data/2007-09-22-12-12-12.
    def load(self):

        self.model = tf.keras.models.load_model(self.model_path, safe_mode=False)
        """
        from keras.models import model_from_json
        print('self.model_path',self.model_path)
        self.model = model_from_json(open(self.model_path+'.json').read())
        self.model.load_weights(self.model_path+'.h5')
        self._compile()
        """

    ###########################################################################
    #
    # show summary
    def summary(self):
        self.model.summary()

