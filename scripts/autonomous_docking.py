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

import signal

# CONSTANTS
OFFSET = 250
MIN_ERROR = 10

global_stop_flag = False

xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.112'

right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

rospy.init_node('robot_controller', anonymous=True)

#Setpoint [rad/s]
#W miarę OK
Kp = 14
Ki = 7
Kd = 0.3

# RIGHT WHEEL PID
pid_right = PID(Kp, Ki, Kd, setpoint=0)
pid_right.sample_time = 0.01
pid_right.output_limits = (-1000, 1000)

# LEFT WHEEL PID
pid_left = PID(Kp, Ki, Kd, setpoint=0)
pid_left.sample_time = 0.01
pid_left.output_limits = (-1000, 1000)

# ALIGNMENT PID
pid_align = PID(1.5, 0.2, 0.008)
pid_align.sample_time = 0.01
pid_align.setpoint = 0
pid_align.output_limits = (-15, 15)

# Read values from topic
right_wheel_speed = 0
left_wheel_speed = 0

rospy.init_node('robot_controller', anonymous=True)

tfmini_measurements = [0, 0, 0, 0]
pololu_measurements = [0, 0, 0, 0]

frame_size = 10
average_tfmini = [[0 for i in range(frame_size)] for _ in range(4)]

def precise_tfmini_measurement():
    '''
    Process data from tfmini sensors using a moving average filter.
    '''
    
    precise_tfmini = [0, 0, 0, 0]

    for i in range(4):
        average_tfmini[i].pop(0)
        average_tfmini[i].append(tfmini_measurements[i])

        for j in range(frame_size):
            precise_tfmini[i] += average_tfmini[i][j]/frame_size
            precise_tfmini[i] = int(precise_tfmini[i])

    # print(precise_tfmini)
    return precise_tfmini    

def controller_manager_setup():
    command = f'{xavier_setup}'
    command += ' && rosservice call /robot_driver/motor_contoller/turn_off_PID True'
    command += ' && rosrun controller_manager controller_manager stop diff_drive'
    command += ' && rosrun controller_manager controller_manager unload diff_drive'
    command += ' && rosrun controller_manager controller_manager spawn right_velocity'
    command += ' && rosrun controller_manager controller_manager spawn left_velocity'
    p1=Popen(['/bin/bash', '-i', '-c', command])

    p1.wait(timeout=60)

    while not global_stop_flag: pass
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

    global pid_right, right_wheel_speed

    pid_right.setpoint=speed
    output = pid_right(right_wheel_speed)

    if output > 0: output += OFFSET
    elif output < 0: output -= OFFSET

    right_wheel_publisher.publish(output)

def move_left_wheel(speed):
    '''
    Move left wheel using PID controller.
    '''

    global pid_left, left_wheel_speed 
    pid_left.setpoint=speed
    output = pid_left(left_wheel_speed)

    if output > 0: output += OFFSET
    elif output < 0: output -= OFFSET
    
    left_wheel_publisher.publish(output)

def full_stop():
    '''
    Stop all motors, by sending 0% PWM signal. 
    '''

    right_wheel_publisher.publish(0)
    left_wheel_publisher.publish(0)

def align_robot(sensor='tfmini', precision=False):
    '''
    Put robot in parallel position to the wall. 
    '''

    global pid_align

    if sensor=='tfmini':
        if precision:
            precise_tfmini = precise_tfmini_measurement()
            error = precise_tfmini[3] - precise_tfmini[2]
        else:
            error = tfmini_measurements[3] - tfmini_measurements[2]
            
        if -MIN_ERROR <= error <= MIN_ERROR: error = 0

    elif sensor=='pololu':
        error = pololu_measurements[3] - pololu_measurements[2] 
        if -MIN_ERROR <= error <= MIN_ERROR: error = 0

    
    # print('tf2', tfmini_measurements[2], end='\t')
    # print('tf3', tfmini_measurements[3], end='\t')

    pid_align.setpoint=0
    output_align = pid_align(error)

    # move_right_wheel(output_align)

    print(f'{output_align = }, {error=}')

    return output_align, error

def find_docking_wall():
    '''
    Based on the distance of the front lidar, specify if the robot
    is close enough to the wall to start the alignment process. 
    '''

    pass

def docking():
    pass 

def signal_handler(signal, frame):
    global global_stop_flag
    global_stop_flag = True

    
if __name__ == '__main__':

    #Use right_velocity and left_velocity instead of diff_drive
    #Disconnect built in PID controller
    # controller_manager_setup()

    signal.signal(signal.SIGINT, signal_handler)

    #Start listener thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.setDaemon(True)
    listener_thread.start()

    while True:
        alignment_movement, alignment_error = align_robot()
        try:
            forward_movement = 5 / abs(alignment_error)
        except ZeroDivisionError:
            forward_movement = 5
        # move_left_wheel(forward_movement - alignment_movement)
        # move_right_wheel(forward_movement)
        # align_robot()
        full_stop()
        # right_wheel_publisher.publish(0)
        # precise_tfmini_measurement()

        if global_stop_flag:
            break

    print('debug message')
    full_stop()
    #rospy.signal_shutdown('Finished Docking')
    listener_thread.join()
    print('that\'s all folks, end of the story')