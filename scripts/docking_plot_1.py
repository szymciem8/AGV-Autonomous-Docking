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

def clear_data(df):
        start_time = df.iloc[0,7]
        df = df[df['distance[mm]'] > 150.0]
        df['time[s]'] = df['time[s]'] - start_time
        df = df.apply(pd.to_numeric)

        return df

# path = os.path.join(os.path.dirname(__file__), 'logs/500/ride_0_base_speed_2.8_without_rosbag.csv')
file = 'ride_14'
path = os.path.join(os.path.dirname(__file__), 'logs/measurements/' + file + '.csv')
df = pd.read_csv(path)

df = clear_data(df)

start_time = df.iloc[0,7]
docking_time = df.iloc[-3,1]
accurate_distance = df.iloc[-2,1]
base_speed = df.iloc[-1,1]


# Delete two last rows
# df = df.iloc[:-1]

# df['distance[mm]'] = get_distance_from_wall(df['front[mm]'], df['rear[mm]'], df['angle[rad]'])
# df['angle[rad]'] = get_angle(df['error[mm]'])

time = df['time[s]'].values.tolist()
distance = df['distance[mm]'].values.tolist()
angle = df['angle[rad]'].values.tolist()
pid_distance_setpoint = df['PID Distance setpoint'].values.tolist()
pid_align_setpoint = df['PID Align setpoint'].values.tolist()

min_distance = (df['PID Distance setpoint'] + 50).values.tolist()
max_distance = (df['PID Distance setpoint'] - 50).values.tolist()

fig, axs = plt.subplots(2)
axs[0].yaxis.set_ticks(np.arange(100, 1100, 20))
axs[0].xaxis.set_ticks(np.arange(0, 20, 1))
axs[0].set_xlabel('Time[s]')
axs[0].set_ylabel('Distance from wall [mm]')
axs[0].fill_between(time, min_distance, max_distance, facecolor='green', alpha=0.2)
axs[0].plot(time, distance)
axs[0].plot(time, pid_distance_setpoint)
axs[0].grid()
axs[0].legend(['Distance', 'Distance setpoint', 'Acceptable error'])

angle_from_error = get_angle(10)
min_angle = list([angle_from_error] * len(time))
max_angle = [-angle_from_error] * len(time)

axs[1].yaxis.set_ticks(np.arange(-0.6, 0.6, 0.05))
axs[1].xaxis.set_ticks(np.arange(0, 20, 1))
axs[1].set_xlabel('Time[s]')
axs[1].set_ylabel('Robot angle against the wall [rad]')
axs[1].fill_between(time, min_angle, max_angle, facecolor='green', alpha=0.2)
axs[1].plot(time, angle, 'tab:green')
axs[1].plot(time, pid_align_setpoint, 'tab:red')
axs[1].grid()
axs[1].legend(['Actual angle', 'Angle setpoint', 'Acceptable angle'])

fig.set_size_inches(15,9)
plt.tight_layout()      
plt.savefig('images/from_logs/'+ file +'.png', dpi=200)
plt.show()

# path = os.path.join(os.path.dirname(__file__), 'images/from_logs/' + file + '.png')


# path = os.path.join(os.path.dirname(__file__), 'logs/new_500/' + file + '.csv')
# df.to_csv(path, index=False)

# with open(path, 'a') as f:
#     writer = csv.writer(f)
#     writer.writerow(['Full Time'] + [docking_time] + list(np.zeros(6)))
#     writer.writerow(['Full Distance'] + [accurate_distance] + list(np.zeros(6)))