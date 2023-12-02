import tensorflow as tf
from keras.backend.tensorflow_backend import set_session

# Assuming you have a GPU with device ID 0
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.gpu_options.visible_device_list = "0"  # Set the GPU device ID
set_session(tf.Session(config=config))

# Your Keras code here...

# Reset GPU memory
tf.keras.backend.clear_session()
