import pandas as pd

import tensorflow as tf

from tensorflow import keras
from tensorflow.keras import layers

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 


def predict_docking_distance():
    data = {'base_speed':[2.5], 
    'distance_from_wall':[350], 
    'rotation_angle':[0.2], 
    'distance_setpoint':[500.0]}

    current_df = pd.DataFrame(data)

    return nn_model.predict(current_df)[0][0]

nn_model = keras.models.load_model('/home/ubuntu/catkin_ws/src/AGV-Autonomous-Docking/docking_prediction/docking_distance_prediction/')

print(predict_docking_distance())