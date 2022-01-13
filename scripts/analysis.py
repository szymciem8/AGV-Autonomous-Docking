from codecs import ignore_errors
import matplotlib.pyplot as plt
import pandas as pd
import os
import csv
import numpy as np

def get_angle(error):
        '''
        Calculate angle of a robot against the wall
        '''
        D = 195
        return np.arcsin(error/np.sqrt(D**2+error**2))

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
file = 'test'
path = os.path.join(os.path.dirname(__file__), 'logs/500/' + file + '.csv')
df = pd.read_csv(path)

start_time = df.iloc[0,7]
docking_time = df.iloc[-2,1]
accurate_distance = df.iloc[-1,1]

# Delete two last rows
df = df.iloc[:-2]

df = df.apply(pd.to_numeric)

df['distance[mm]'] = get_distance_from_wall(df['front[mm]'], df['rear[mm]'], df['angle[rad]'])
df['time[s]'] = df['time[s]'] - start_time
df = df[df['distance[mm]'] > 399.0]

df['angle[rad]'] = get_angle(df['error[mm]'])*1000

ax = plt.gca()

df.plot(kind='line', x='time[s]', y='distance[mm]', ax=ax)
df.plot(kind='line', x='time[s]', y='angle[rad]', ax=ax)
df.plot(kind='line', x='time[s]', y='PID Distance setpoint', ax=ax)
df.plot(kind='line', x='time[s]', y='PID Align setpoint', ax=ax)


plt.rcParams['figure.figsize'] = (5,5)

plt.show()

# path = os.path.join(os.path.dirname(__file__), 'images/from_logs/' + file + '.png')
plt.savefig('images/from_logs/test.png')

path = os.path.join(os.path.dirname(__file__), 'logs/new_500/' + file + '.csv')
df.to_csv(path, index=False)

with open(path, 'a') as f:
    writer = csv.writer(f)
    writer.writerow(['Full Distance'] + [5] + list(np.zeros(6)))