#!/usr/bin/env python

from logging import error
from numpy.core.numeric import full
from geometry_msgs import msg
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu, Range

from simple_pid import PID
import threading
import matplotlib.pyplot as plt
from subprocess import call, Popen
import time
import math
import csv

import signal

from agv_filters import MedianFilter, MovingAverageFilter

# CONSTANTS
OFFSET = 455
MIN_ERROR = 15
MIN_SPEED = 2.5

f = open('agv_log.csv', 'w')
writer = csv.writer(f)

header = ['pid_align', 'output_align', 'error', 'angle', 'distance']
writer.writerow(header)

# pololu_mf = MedianFilter(5)
# tfmini_mf = MedianFilter(5)
pololu_mf = MedianFilter(window_width=5, num_sensors=4)
tfmini_mf = MedianFilter(window_width=5, num_sensors=4)

global_stop_flag = False

xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.112'

right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

rospy.init_node('robot_controller', anonymous=True)

#Setpoint [rad/s]
#W miarę OK
# Kp = 7
# Ki = 15
# Kd = 0.5

# RIGHT WHEEL PID
pid_right = PID(6, 5, 0.1, setpoint=0)
pid_right.sample_time = 0.001
pid_right.output_limits = (-1000, 1000)

# LEFT WHEEL PID
pid_left = PID(5, 5, 0.1, setpoint=0)
pid_left.sample_time = 0.001
pid_left.output_limits = (-1000, 1000)

# ALIGNMENT PID
# It controls alignment of a robota against the wall
# Both setpoint and output of a pid are angles

# pid_align = PID(2.5, 0.4, 0.008)
pid_align = PID(2, 0.5, 0.008)
pid_align.sample_time = 0.001
pid_align.setpoint = -0.6
pid_align.output_limits = (-20, 20)

# DISTANCE PID
# It controls distance of robot from the wall. 
# Setpoint is a distnce, and output is an angle. 

pid_distance = PID(0.001, 0, 0)
pid_distance.sample_time = 0.001
pid_distance.output_limits = (-0.3, 0.3)

# Read values from topic
right_wheel_speed = 0
left_wheel_speed = 0

rospy.init_node('robot_controller', anonymous=True)

tfmini_measurements = [0, 0, 0, 0]
pololu_measurements = [0, 0, 0, 0]


def controller_manager_setup():
    command = f'{xavier_setup}'
    command += ' && rosservice call /robot_driver/motor_contoller/turn_off_PID True'
    command += ' && rosrun controller_manager controller_manager stop diff_drive'
    command += ' && rosrun controller_manager controller_manager unload diff_drive'
    command += ' && rosrun controller_manager controller_manager spawn right_velocity'
    command += ' && rosrun controller_manager controller_manager spawn left_velocity'
    p1=Popen(['/bin/bash', '-i', '-c', command])

    p1.wait(timeout=60)
    p1.terminate()

def callback_pololu(msg, i):
    global pololu_measurements
    pololu_measurements[i] = float(msg.range)
    # print(pololu_measurements)

def callback_tfmini(msg, i):
    global tfmini_measurements
    tfmini_measurements[i] = float(msg.range)
    #print(tfmini_measurements)

def callback_joint_state(msg):
    global right_wheel_speed, left_wheel_speed
    right_wheel_speed = msg.velocity[1]
    left_wheel_speed = msg.velocity[0]
    # print(right_wheel_speed, left_wheel_speed)

def listener():
    global global_stop_flag

    rospy.Subscriber('/joint_states', JointState, callback_joint_state)
    for i in range(4):
        rospy.Subscriber('/mega_driver/pololu/scan_'+str(i), Range, callback_pololu, callback_args=i)
        rospy.Subscriber('/mega_driver/tfmini/scan_'+str(i), Range, callback_tfmini, callback_args=i)

    while not global_stop_flag: pass
    # rospy.spin()

def move_right_wheel(speed):
    '''
    Move right wheel using PID controller.
    '''
    RIGHT_OFFSET = 450

    global pid_right, right_wheel_speed

    if speed < MIN_SPEED and speed > 0: speed=MIN_SPEED
    elif speed > -MIN_SPEED and speed < 0: speed=-MIN_SPEED

    pid_right.setpoint=speed
    output = pid_right(right_wheel_speed)

    if speed > 0 and output < 0:
        output=0
    elif speed < 0 and output > 0:
        output=0

    if output > 0: output += RIGHT_OFFSET
    elif output < 0: output -= RIGHT_OFFSET
    right_wheel_publisher.publish(output)

def move_left_wheel(speed):
    '''
    Move left wheel using PID controller.
    '''

    LEFT_OFFSET = 480

    global pid_left, left_wheel_speed

    if speed < MIN_SPEED and speed > 0: speed=MIN_SPEED
    elif speed > -MIN_SPEED and speed < 0: speed=-MIN_SPEED

    pid_left.setpoint=speed
    output = pid_left(left_wheel_speed)

    if speed > 0 and output < 0:
        output=0
    elif speed < 0 and output > 0:
        output=0

    if output > 0: output += LEFT_OFFSET
    elif output < 0: output -= LEFT_OFFSET

    # print(f'{output=}')
    left_wheel_publisher.publish(output)

def full_stop():
    '''
    Stop all motors, by sending 0% PWM signal.
    '''

    right_wheel_publisher.publish(0)
    left_wheel_publisher.publish(0)

def get_angle(error):
    D = 195

    return math.asin(error/math.sqrt(D**2+error**2))

def get_distance_from_wall(l1, l2, angle):
    # In mm
    X0 = 150   #X0 = 150
    Y0 = 87    #Y0 = 85

    # d/l1 = cos(alfa) - > d = li

    angle = get_angle(l2-l1)

    d = l1 * math.cos(angle)

    return d + math.cos(angle) * X0 - math.sin(angle) * Y0

def dead_space(value, cutter):
    '''
    Exclude dead space from given value and its cutter.
    '''

    if value > cutter: value=cutter
    elif value < cutter: value=cutter
    return value

def align_robot(sensor='tfmini', precision=False):
    '''
    Put robot in parallel position to the wall.
    '''

    global pid_align
    global pid_distance

    global pololu_mf
    global tfmini_mf

    if sensor=='tfmini':
        if precision:
            tfmini_mf.update_measurements(tfmini_measurements)
            precise_tfmini = tfmini_mf.output()
            error = precise_tfmini[3] - precise_tfmini[2]
        else:
            error = tfmini_measurements[3] - tfmini_measurements[2] # Rear - front

        if -MIN_ERROR <= error <= MIN_ERROR: error = 0

        # distance = get_distance_from_wall(pololu_measurements[2], tfmini_measurements[3], error)

    elif sensor=='pololu':
        if precision:
            pololu_mf.update_measurements(pololu_measurements)
            precise_pololu = pololu_mf.output()
            error = precise_pololu[3] - precise_pololu[2]
        else:
            error = pololu_measurements[3] - pololu_measurements[2] # Rear - front

        # if -MIN_ERROR <= error <= MIN_ERROR: error = 0

    # print('tf2', pololu_measurements[2], end='\t')
    # print('tf3', pololu_measurements[3], end='\t')
    # print('difference', pololu_measurements[3] - pololu_measurements[2])

    angle = get_angle(error)
    distance = get_distance_from_wall(pololu_measurements[2], pololu_measurements[3], angle)

    pid_distance.setpoint = 500

    if 480 < distance < 520:
        pid_align.setpoint = 0
    else:
        pid_align.setpoint = -pid_distance(distance)

    output_align = pid_align(angle)

    # if output_align < 0: output_align -=5
    # elif output_align >0: output_align += 5

    row = f'{pid_align.setpoint=}, {output_align=}, {error=}, {angle=}, {distance=}'
    print(row)
    writer.writerow([pid_align.setpoint, output_align, error, angle, distance])

    return output_align, error, angle

def find_docking_wall():
    '''
    Based on the distance from the front lidar, specify if the robot
    is close enough to the wall to start the alignment process.
    '''

    pass

def docking():
    pass

def signal_handler(signal, frame):
    global global_stop_flag
    global_stop_flag = True


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    #Use right_velocity and left_velocity instead of diff_drive
    #Disconnect built in PID controller
    # controller_manager_setup()

    #Start listener thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.setDaemon(True)
    listener_thread.start()

    full_stop()
    while True:
        alignment_movement, alignment_error, alignment_angle = align_robot('pololu', True)

        try:
            forward_movement = 5 / abs(alignment_error)
        except ZeroDivisionError:
             forward_movement = 5

        # print(get_angle(alignment_error))
        # print(get_distance_from_wall(pololu_measurements[2], pololu_measurements[3], alignment_angle))

        # move_left_wheel(abs(alignment_movement))
        # move_right_wheel(alignment_movement)

        base_speed = 2.8
        if alignment_angle > pid_align.setpoint:
            move_right_wheel(base_speed)
            move_left_wheel(base_speed + abs(alignment_movement))
        elif alignment_angle < pid_align.setpoint:
            move_right_wheel(base_speed + abs(alignment_movement))
            move_left_wheel(base_speed)
        else:
            move_right_wheel(base_speed)
            move_left_wheel(base_speed)


        # move_right_wheel(alignment_movement)
        # move_left_wheel(-alignment_movement)

        if global_stop_flag:
            break

    print('debug message')
    full_stop()
    #rospy.signal_shutdown('Finished Docking')
    listener_thread.join()
    f.close()
    print('that\'s all folks, end of the story')