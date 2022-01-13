import matplotlib.pyplot as plt
import pandas as pd
import os
import math
import numpy as np

def get_angle(error):
        '''
        Calculate angle of a robot against the wall
        '''
        D = 195
        return math.asin(error/math.sqrt(D**2+error**2))

def get_distance_from_wall(l1, l2, angle):
        '''
        Calculate the distance of the point on the robot placed between the wheels
        '''
        # In mm
        X0 = 150
        Y0 = 87

        # d/l1 = cos(alfa) -> d = li

        d = l1 * np.cos(angle)
        return d + np.cos(angle) * X0 - np.sin(angle) * Y0

# path = os.path.join(os.path.dirname(__file__), 'logs/500/ride_0_base_speed_2.8_without_rosbag.csv')
path = os.path.join(os.path.dirname(__file__), 'logs/500/test.csv')
df = pd.read_csv(path)

docking_time = df.iloc[-2,1]
accurate_distance = df.iloc[-1,1]

# Delete two last rows
df = df.iloc[:-2]

df = df.apply(pd.to_numeric)

df['distance[mm]'] = get_distance_from_wall(df['front[mm]'], df['rear[mm]'], df['angle[rad]'])

# df = df[df['distance[mm]'] != 150.0]
print(df.head())

ax = plt.gca()

df.plot(kind='line', x='time[s]', y='distance[mm]', ax=ax)

plt.show()