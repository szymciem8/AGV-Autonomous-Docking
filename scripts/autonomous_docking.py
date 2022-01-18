#!/usr/bin/env python

# Copyright 2022 Szymon Ciemała, Paweł Wojaczek

from logging import error
from numpy.core.numeric import full
from geometry_msgs import msg
import rospy
from rospy.exceptions import ROSTimeMovedBackwardsException
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu, Range

from simple_pid import PID
import threading
import matplotlib.pyplot as plt
from subprocess import call, Popen
import time
import math
import csv
import os

import rosbag
import signal

from agv_filters import MedianFilter, MovingAverageFilter

class AGV:

    def __init__(self, file_name):

        # CONSTANTS
        self.OFFSET = 455
        self.MIN_ERROR = 15
        self.MIN_SPEED = 2.5

        path = os.path.join(os.path.dirname(__file__), 'logs/' + file_name +'.csv')
        self.f = open(path, 'w')
        self.writer = csv.writer(self.f)

        self.pololu_mf = MedianFilter(window_width=5, num_sensors=4)
        self.tfmini_mf = MedianFilter(window_width=5, num_sensors=4)

        self.global_stop_flag = False

        self.xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.109'

        self.right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
        self.left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

        rospy.init_node('robot_controller', anonymous=True)

        # RIGHT WHEEL PID
        # Setpoint [rad/s]
        self.pid_right = PID(6, 5, 0.1, setpoint=0)
        self.pid_right.sample_time = 0.001
        self.pid_right.output_limits = (-1000, 1000)

        # LEFT WHEEL PID
        self.pid_left = PID(5, 5, 0.1, setpoint=0)
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
        self.pololu_measurements = [0, 0, 0, 0]

    def __del__(self):
        self.f.close()

    def controller_manager_setup(self):
        '''
        Disable differential drive controller and enable separate controllers for each wheel. 
        '''

        command = f'{self.xavier_setup}'
        command += ' && rosservice call /robot_driver/motor_contoller/turn_off_PID True'
        command += ' && rosrun controller_manager controller_manager stop diff_drive'
        command += ' && rosrun controller_manager controller_manager unload diff_drive'
        command += ' && rosrun controller_manager controller_manager spawn right_velocity'
        command += ' && rosrun controller_manager controller_manager spawn left_velocity'
        p1=Popen(['/bin/bash', '-i', '-c', command])

        p1.wait(timeout=60)
        p1.terminate()

    def callback_pololu(self, msg, i):
        '''
        Read pololu data from ROS topic
        '''
        self.pololu_measurements[i] = float(msg.range)

    def callback_tfmini(self, msg, i):
        '''
        Read tfmini data from ROS topic
        '''
        self.tfmini_measurements[i] = float(msg.range)

    def callback_joint_state(self, msg):
        '''
        Get speed values [rad/s] of each wheel. 
        '''
        self.right_wheel_speed = msg.velocity[1]
        self.left_wheel_speed = msg.velocity[0]

    def listener(self):
        '''
        Read data from given topics and send save them to choosen variables
        '''

        rospy.Subscriber('/joint_states', JointState, self.callback_joint_state)
        for i in range(4):
            rospy.Subscriber('/mega_driver/pololu/scan_'+str(i), Range, self.callback_pololu, callback_args=i)
            rospy.Subscriber('/mega_driver/tfmini/scan_'+str(i), Range, self.callback_tfmini, callback_args=i)

        while not self.global_stop_flag: pass

    def move_right_wheel(self, speed):
        '''
        Move right wheel using PID controller.
        '''
        RIGHT_OFFSET = 450

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

        LEFT_OFFSET = 480

        global pid_left, left_wheel_speed

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

    def full_stop(self):
        '''
        Stop all motors, by sending 0% PWM signal.
        '''

        self.right_wheel_publisher.publish(0)
        self.left_wheel_publisher.publish(0)

    def get_angle(self, error):
        '''
        Calculate angle of a robot against the wall
        '''
        D = 195
        return math.asin(error/math.sqrt(D**2+error**2))

    def get_distance_from_wall(self, l1, l2):
        '''
        Calculate the distance of the point on the robot placed between the wheels
        '''
        # In mm
        X0 = 150
        Y0 = 87

        # d/l1 = cos(alfa) -> d = li

        angle = self.get_angle(l2-l1)

        d = l1 * math.cos(angle)

        return d + math.cos(angle) * X0 - math.sin(angle) * Y0

    def dead_space(value, cutter):
        '''
        Exclude dead space from given value and its cutter.
        '''

        if value > cutter: value=cutter
        elif value < cutter: value=cutter
        return value

    def align_robot(self, set_distance, distance_error, sensor='tfmini', precision=False):
        '''
        Put the robot in a position parallel to the wall.
        '''
        
        if sensor=='tfmini':
            if precision:
                self.tfmini_mf.update_measurements(self.tfmini_measurements)
                precise_tfmini = self.tfmini_mf.output()
                self.error = precise_tfmini[3] - precise_tfmini[2]
            else:
                self.error = self.tfmini_measurements[3] - self.tfmini_measurements[2] # Rear - front

            if -self.MIN_ERROR <= self.error <= self.MIN_ERROR: self.error = 0

            # distance = get_distance_from_wall(pololu_measurements[2], tfmini_measurements[3])

        elif sensor=='pololu':
            if precision:
                self.pololu_mf.update_measurements(self.pololu_measurements)
                self.precise_pololu = self.pololu_mf.output()
                self.error = self.precise_pololu[3] - self.precise_pololu[2]
            else:
                self.error = self.pololu_measurements[3] - self.pololu_measurements[2] # Rear - front

            # if -MIN_ERROR <= error <= MIN_ERROR: error = 0

        self.angle = self.get_angle(self.error)
        self.distance = self.get_distance_from_wall(self.pololu_measurements[2], self.pololu_measurements[3])

        self.pid_distance.setpoint = set_distance

        if set_distance * (1 - distance_error/100) < self.distance < set_distance * (1 + distance_error/100):
            self.pid_align.setpoint = 0

            if -10 < self.error < 10:
                self.global_stop_flag = True
        else:
            self.pid_align.setpoint = -self.pid_distance(self.distance)

        output_align = self.pid_align(self.angle)

        # if output_align < 0: output_align -=5
        # elif output_align >0: output_align += 5

        # print(f'{self.pid_align.setpoint=}, {output_align=}, {error=}, {angle=}, {distance=}')
        # self.writer.writerow([self.pid_align.setpoint, output_align, error, angle, distance])

        return output_align, self.error, self.angle

    def docking(self, base_speed=3, set_distance=500):
        '''
        Based on calculations (PID Controlers, angles etc.) guide the robot to the predifined
        docking position. 
        '''

        alignment_movement, alignment_error, alignment_angle = self.align_robot(set_distance, 10, 'pololu', True)

        if alignment_angle > self.pid_align.setpoint:
            self.move_right_wheel(base_speed)
            self.move_left_wheel(base_speed + abs(alignment_movement))
        elif alignment_angle < self.pid_align.setpoint:
            self.move_right_wheel(base_speed + abs(alignment_movement))
            self.move_left_wheel(base_speed)
        else:
            self.move_right_wheel(base_speed)
            self.move_left_wheel(base_speed)

    def save_to_csv(self, data):

        if data:
            self.writer.writerow(data)
        else:
            self.writer.writerow([self.error, self.angle, self.distance])

    def find_docking_wall(self):
        '''
        Based on the distance from the front lidar, specify if the robot
        is close enough to the wall to start the alignment process.
        '''

        pass

def signal_handler(signal, frame):
    global robot
    robot.global_stop_flag = True


if __name__ == '__main__':

    n = '49'
    base_speed = 2.5
    rbag = 'without_rosbag'
    set_distance = 500

    # robot = AGV(str(set_distance)+ '/' + 'ride_' + n + '_base_speed_' + str(base_speed) + '_' + rbag)

    robot = AGV('measurements/ride_'+n)

    signal.signal(signal.SIGINT, signal_handler)
    # robot.controller_manager_setup()

    # Start listener thread
    listener_thread = threading.Thread(target=robot.listener)
    listener_thread.setDaemon(True)
    listener_thread.start()

    robot.full_stop()
    robot.save_to_csv(['front[mm]', 
                       'rear[mm]', 
                       'PID Align setpoint', 
                       'PID Distance setpoint', 
                       'error[mm]', 
                       'angle[rad]', 
                       'distance[mm]', 
                       'time[s]', 
                       'rw_speed[rad/s]', 
                       'lw_speeed[rad/s]'])

    topics = rospy.get_published_topics()

    time.sleep(2)

    i=0
    start = time.time()
    while True:
        i+=1

        robot.docking(base_speed, 500)

        if i > 2:
            robot.save_to_csv([robot.precise_pololu[2], 
                            robot.precise_pololu[3], 
                            robot.pid_align.setpoint, 
                            robot.pid_distance.setpoint, 
                            robot.error, 
                            robot.angle, 
                            robot.distance, 
                            time.time(), 
                            robot.right_wheel_speed,
                            robot.left_wheel_speed])
            
        if i < 3:
            robot.global_stop_flag=False

        if robot.global_stop_flag:
            break

    end = time.time()

    robot.full_stop()

    robot.save_to_csv(['Full Time',end-start])
    full_distance = input('Full distance: ')

    robot.save_to_csv(['Full Distance', full_distance])
    robot.save_to_csv(['Base Speed', base_speed])
    listener_thread.join()

    # bag.close() 

    del robot
    print('that\'s all folks, end of the story')