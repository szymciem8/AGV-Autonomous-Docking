from codecs import ignore_errors
from dis import dis
from cv2 import displayStatusBar, divide
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

# path = os.path.join(os.path.dirname(__file__), 'logs/500/ride_0_base_speed_2.8_without_rosbag.csv')
file = 'ride0'
path = os.path.join(os.path.dirname(__file__), 'logs/portenta/' + file + '.csv')
df_procedure = pd.read_csv(path, skiprows=[0,1,2, 3], names=['time[s]',
                                                          'front[mm]', 
                                                          'rear[mm]', 
                                                          'PID Align setpoint', 
                                                          'PID Distance setpoint', 
                                                          'error[mm]', 
                                                          'angle[rad]', 
                                                          'distance[mm]', 
                                                          'rw_speed[rad/s]', 
                                                          'lw_speeed[rad/s]', 
                                                          'pozyx_x_1', 
                                                          'pozyx_y_1',
                                                          'pozyx_rot_y_1', 
                                                          'pozyx_x_2',
                                                          'pozyx_y_2',
                                                          'pozyx_rot_2'
                                                        ])
df_single = pd.read_csv(path, skiprows=lambda x: x>2, index_col=0, header=None).T

start_time = df_procedure.iloc[0,0]
docking_time = df_single.iloc[0,0]
docking_distance = df_single.iloc[0,1]
base_speed = df_single.iloc[0,2]

df_procedure['time[s]'] = df_procedure['time[s]'] - start_time
print(df_procedure['time[s]'])

time = df_procedure['time[s]'].values.tolist()
# df_procedure['distance[mm]'] = df_procedure['distance[mm]'].interpolate()
distance = df_procedure['distance[mm]'].values.tolist()
angle = df_procedure['angle[rad]'].values.tolist()
pid_distance_setpoint = df_procedure['PID Distance setpoint'].values.tolist()
pid_align_setpoint = df_procedure['PID Align setpoint'].values.tolist()
pozyx_x_1 = df_procedure['pozyx_x_1'].values.tolist()

min_distance = (df_procedure['PID Distance setpoint'] + 50).values.tolist()
max_distance = (df_procedure['PID Distance setpoint'] - 50).values.tolist()

fig, axs = plt.subplots(2)
axs[0].yaxis.set_ticks(np.arange(100, 1100, 20))
axs[0].xaxis.set_ticks(np.arange(0, 20, 1))
axs[0].set_xlabel('Time[s]')
axs[0].set_ylabel('Distance from wall [mm]')
axs[0].fill_between(time, min_distance, max_distance, facecolor='green', alpha=0.2)
axs[0].plot(time, distance)
axs[0].plot(time, pid_distance_setpoint)
axs[0].plot(time, pozyx_x_1)
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
plt.savefig('images/from_logs/portenta/'+ file +'.png', dpi=200)
# plt.show()

# path = os.path.join(os.path.dirname(__file__), 'images/from_logs/' + file + '.png')

# # path = os.path.join(os.path.dirname(__file__), 'logs/new_500/' + file + '.csv')
# # df.to_csv(path, index=False)

# # with open(path, 'a') as f:
# #     writer = csv.writer(f)
# #     writer.writerow(['Full Time'] + [docking_time] + list(np.zeros(6)))
# #     writer.writerow(['Full Distance'] + [docking_distance] + list(np.zeros(6)))