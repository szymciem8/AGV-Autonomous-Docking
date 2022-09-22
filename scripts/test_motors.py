#!/usr/bin/env python

# Copyright 2022 Szymon Ciemała, Paweł Wojaczek

from codecs import ignore_errors
from logging import error, root
from numpy.core.numeric import full
from geometry_msgs import msg 
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from rospy.exceptions import ROSTimeMovedBackwardsException
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState, Imu, Range, LaserScan

from simple_pid import PID
import threading
import matplotlib.pyplot as plt
from subprocess import call, Popen
import time
import math
import csv
import os

import pandas as pd

import tensorflow as tf

# from tensorflow import keras
# from tensorflow.keras import layers

import rosbag
import signal

from agv_filters import MedianFilter, MovingAverageFilter

class AGV:

    def __init__(self, file_name):

        # CONSTANTS
        self.OFFSET = 455
        self.MIN_ERROR = 15
        # self.MIN_SPEED = 2.5
        self.MIN_SPEED = 0

        path = os.path.join(os.path.dirname(__file__), 'logs/' + file_name +'.csv')
        self.f = open(path, 'w')
        self.writer = csv.writer(self.f)

        self.pololu_mf = MedianFilter(window_width=5, num_sensors=8)

        self.global_stop_flag = False

        self.xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.109'

        self.control_mode = rospy.Publisher('change_topic', String, queue_size=10)
        self.right_wheel_publisher = rospy.Publisher('control_right', Float64, queue_size=10)
        self.left_wheel_publisher = rospy.Publisher('control_left', Float64, queue_size=10)

        rospy.init_node('robot_controller', anonymous=True)

        # Distance sensors
        self.front_left_dis = 0
        self.rear_left_dis = 0

        # Read values from topic
        self.right_wheel_speed = 0
        self.left_wheel_speed = 0

        self.error = 0
        self.distance = 0
        self.angle = 0

        self.precise_tfmini = None
        self.precise_pololu = None

        rospy.init_node('robot_controller', anonymous=True)

        self.tfmini_measurements = [0, 0, 0, 0]
        self.pololu_measurements = [0, 0, 0, 0, 0, 0, 0, 0]

        # Pozyx
        self.pozyx_x_1 = 0
        self.pozyx_y_1 = 0
        self.pozyx_rot_y_1 = 0

        self.pozyx_x_2 = 0
        self.pozyx_y_2 = 0
        self.pozyx_rot_y_2 = 0

        # Lidar
        self.laser_msg = None

        # self.nn_model = keras.models.load_model('/home/ubuntu/catkin_ws/src/AGV-Autonomous-Docking/docking_prediction/docking_distance_prediction/')

    def __del__(self):
        self.f.close()

    def initialize_pid_ctrl(self):
        # RIGHT WHEEL PID
        # Setpoint [rad/s]
        # self.pid_right = PID(6, 5, 0.1, setpoint=0)
        self.pid_right = PID(7, 14, 0.1, setpoint=0)
        self.pid_right.sample_time = 0.001
        self.pid_right.output_limits = (-1000, 1000)

        # LEFT WHEEL PID
        # self.pid_left = PID(5, 5, 0.1, setpoint=0)
        self.pid_left = PID(10, 14, 0.1, setpoint=0)
        self.pid_left.sample_time = 0.001
        self.pid_left.output_limits = (-1000, 1000)

        # ALIGNMENT PID
        # It controls alignment of a robota against the wall
        # Setpoint [rad], output[rad]

        # pid_align = PID(2.5, 0.4, 0.008)
        self.pid_align = PID(3, 0.5, 0.008)
        self.pid_align.sample_time = 0.001
        self.pid_align.setpoint = -0.6
        self.pid_align.output_limits = (-20, 20)

        # DISTANCE PID
        # It controls distance of robot from the wall. 
        # Setpoint [mm], output[rad]

        self.pid_distance = PID(0.001, 0, 0)
        self.pid_distance.sample_time = 0.001
        self.pid_distance.output_limits = (-0.3, 0.3)


    def callback_pololu(self, msg, i):
        '''
        Read pololu data from ROS topic
        '''
        self.pololu_measurements[i] = msg.data


    def callback_joint_state(self, msg):
        '''
        Get speed values [rad/s] of each wheel. 
        '''
        # self.right_wheel_speed = msg.velocity[1]
        # self.left_wheel_speed = msg.velocity[0]

    def callback_right_wheel_speed(self, msg):
        self.right_wheel_speed = msg.data

    def callback_left_wheel_speed(self, msg):
        self.left_wheel_speed = -msg.data

    def listener(self):
        '''
        Read data from given topics and send save them to choosen variables
        '''
        # rospy.Subscriber('/joint_states', JointState, self.callback_joint_state)
        rospy.Subscriber('/Encoder_Left', Float64, self.callback_right_wheel_speed)
        rospy.Subscriber('/Encoder_Right', Float64, self.callback_left_wheel_speed)
        for i in range(8):
            rospy.Subscriber('/Pololu'+str(i), Float64, self.callback_pololu, callback_args=i)
            # rospy.Subscriber('/AGV_driver/tfmini/scan_'+str(i), Range, self.callback_tfmini, callback_args=i)

        while not self.global_stop_flag: pass

    def full_stop(self):
        '''
        Stop all motors, by sending 0% PWM signal.
        '''

        self.right_wheel_publisher.publish(0)
        self.left_wheel_publisher.publish(0)

    def move_right_wheel(self, speed):
        '''
        Move right wheel using PID controller.
        '''
        # RIGHT_OFFSET = 450
        RIGHT_OFFSET = 0

        global pid_right, right_wheel_speed

        if speed < self.MIN_SPEED and speed > 0: speed=self.MIN_SPEED
        elif speed > -self.MIN_SPEED and speed < 0: speed=-self.MIN_SPEED

        self.pid_right.setpoint=speed
        output = self.pid_right(self.right_wheel_speed)

        if speed > 0 and output < 0:
            output=0
        elif speed < 0 and output > 0:
            output=0

        if output > 0: output += RIGHT_OFFSET
        elif output < 0: output -= RIGHT_OFFSET
        self.right_wheel_publisher.publish(output)

    def move_left_wheel(self, speed):
        '''
        Move left wheel using PID controller.
        '''

        # LEFT_OFFSET = 480
        LEFT_OFFSET = 0

        # global pid_left, left_wheel_speed

        if speed < self.MIN_SPEED and speed > 0: speed=self.MIN_SPEED
        elif speed > -self.MIN_SPEED and speed < 0: speed=-self.MIN_SPEED

        self.pid_left.setpoint=speed
        output = self.pid_left(self.left_wheel_speed)

        if speed > 0 and output < 0:
            output=0
        elif speed < 0 and output > 0:
            output=0

        if output > 0: output += LEFT_OFFSET
        elif output < 0: output -= LEFT_OFFSET

        self.left_wheel_publisher.publish(output)

    def docking(self, base_speed=3, set_distance=500):
        '''
        Based on calculations (PID Controlers, angles etc.) guide the robot to the predifined
        docking position. 
        '''

        alignment_movement, alignment_error, alignment_angle = self.align_robot(set_distance, 10, 'pololu', False)

        if alignment_angle > self.pid_align.setpoint:
            self.move_right_wheel(base_speed - abs(alignment_movement))
            self.move_left_wheel(base_speed + abs(alignment_movement))
        elif alignment_angle < self.pid_align.setpoint:
            self.move_right_wheel(base_speed + abs(alignment_movement))
            self.move_left_wheel(base_speed - abs(alignment_movement))
        else:
            self.move_right_wheel(base_speed)
            self.move_left_wheel(base_speed)



def signal_handler(signal, frame):
    global robot
    robot.global_stop_flag = True


if __name__ == '__main__':

    n = '24'
    base_speed = 2.5
    rbag = 'without_rosbag'
    set_distance = 500


    # robot = AGV(str(set_distance)+ '/' + 'ride_' + n + '_base_speed_' + str(base_speed) + '_' + rbag)

    name = '500_2/ride_'+n
    robot = AGV(name)

    # bag = rosbag.Bag('scripts/logs/'+name+'.bag', 'w')

    signal.signal(signal.SIGINT, signal_handler)
    # robot.controller_manager_setup()

    # Start listener thread
    listener_thread = threading.Thread(target=robot.listener)
    listener_thread.setDaemon(True)
    listener_thread.start()

    robot.full_stop()

    robot.initialize_pid_ctrl()
    robot.control_mode.publish('a')
    i=0
    start = time.time()
    while True:
        robot.control_mode.publish('a')
        i+=1

        # robot.docking(base_speed, 500)
        # robot.right_wheel_publisher.publish(100)
        # robot.left_wheel_publisher.publish(100)

        robot.move_left_wheel(2)
        robot.move_right_wheel(2)

        print(f"{robot.right_wheel_speed=}, {robot.left_wheel_speed=}, {robot.front_left_dis=}, {robot.rear_left_dis=}")
            
        if i < 3:
            robot.global_stop_flag=False

        if robot.global_stop_flag:
            break

    end = time.time()

    robot.full_stop()
    listener_thread.join()

    del robot
    print('that\'s all folks, end of the story')