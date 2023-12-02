#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tensorflow as tf
import sys

def main(gpu_id):
    # Assuming you have a GPU with device ID 0
    physical_devices = tf.config.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[gpu_id], True)

    # Your TensorFlow 2.x code here...

    # Reset GPU memory
    tf.keras.backend.clear_session()

if __name__ == '__main__':
    try:
        if (len(sys.argv) == 1):
            exit('Usage:\n$ python {} gpu_id_num'.format(sys.argv[0]))

        main(int(sys.argv[1]))

    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')