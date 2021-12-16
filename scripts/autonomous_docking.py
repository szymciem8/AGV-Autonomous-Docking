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

import signal

# CONSTANTS
OFFSET = 455
MIN_ERROR = 15
MIN_SPEED = 2.5


class MedianFilter:
    def __init__(self, ww=5):
        self.window_width = ww
        self.measurements_1 = [0 for _ in range(ww)]
        self.measurements_2 = [0 for _ in range(ww)]

    def update_measurements(self, m_1, m_2):
        self.measurements_1.pop(0)
        self.measurements_1.append(m_1)
        self.measurements_2.pop(0)
        self.measurements_2.append(m_2)

    def output(self):
        try:
            if self.window_width % 2 == 1:
                res_1 = sorted(self.measurements_1)[self.window_width // 2]
                res_2 = sorted(self.measurements_2)[self.window_width // 2]
            else:
                res_1 = sorted(self.measurements_1)[self.window_width // 2]
                res_1 += sorted(self.measurements_1)[(self.window_width // 2) - 1]
                res_1 /= 2
                res_2 = sorted(self.measurements_2)[self.window_width // 2]
                res_2 += sorted(self.measurements_2)[(self.window_width // 2) - 1]
                res_2 /= 2
        except IndexError:
            res_1, res_2 = None, None

        return (res_1, res_2)

pololu_mf = MedianFilter(5)
tfmini_mf = MedianFilter(5)

global_stop_flag = False

xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.112'

right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

rospy.init_node('robot_controller', anonymous=True)

#Setpoint [rad/s]
#W miarę OK
Kp = 10
Ki = 10
Kd = 0.5

# RIGHT WHEEL PID
pid_right = PID(Kp, Ki, Kd, setpoint=0)
pid_right.sample_time = 0.001
pid_right.output_limits = (-1000, 1000)


# Kp = 10
# Ki = 15
# Kd = 0.5

# LEFT WHEEL PID
pid_left = PID(Kp, Ki, Kd, setpoint=0)
pid_left.sample_time = 0.001
pid_left.output_limits = (-1000, 1000)

# ALIGNMENT PID
# pid_align = PID(1.5, 0.2, 0.008)
pid_align = PID(20, 0, 0)
pid_align.sample_time = 0.001
pid_align.setpoint = 0
pid_align.output_limits = (-20, 20)

# Read values from topic
right_wheel_speed = 0
left_wheel_speed = 0

rospy.init_node('robot_controller', anonymous=True)

tfmini_measurements = [0, 0, 0, 0]
pololu_measurements = [0, 0, 0, 0]

frame_size = 10
average_tfmini = [[0 for i in range(frame_size)] for _ in range(4)]


def precise_measurement():
    '''
    Process data from tfmini sensors using a moving average filter.
    '''
    
    l = [0, 0, 0, 0, 0]

    for i in range(len(l)):
        l[i].pop(0)
        l[i].append(pololu_measurements[i])



        for j in range(frame_size):
            l[i] += average_tfmini[i][j]/frame_size
            l[i] = int(l[i])

    # print(precise_tfmini)
    return l    

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

    LEFT_OFFSET = 450

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

    print(f'{output=}')
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
    global pololu_mf
    global tfmini_mf

    if sensor=='tfmini':
        if precision:
            tfmini_mf.update_measurements(tfmini_measurements[3], tfmini_measurements[2])
            precise_tfmini = tfmini_mf.output()
            error = precise_tfmini[1] - precise_tfmini[0]
        else:
            error = tfmini_measurements[3] - tfmini_measurements[2] # Rear - front
            
        if -MIN_ERROR <= error <= MIN_ERROR: error = 0

    elif sensor=='pololu':
        if precision:
            pololu_mf.update_measurements(pololu_measurements[3], pololu_measurements[2])
            precise_pololu = pololu_mf.output()
            error = precise_pololu[1] - precise_pololu[0]
        else:
            error = pololu_measurements[3] - pololu_measurements[2] # Rear - front

        # if -MIN_ERROR <= error <= MIN_ERROR: error = 0

    
    # print('tf2', pololu_measurements[2], end='\t')
    # print('tf3', pololu_measurements[3], end='\t')
    # print('difference', pololu_measurements[3] - pololu_measurements[2])

    angle = get_angle(error)
    pid_align.setpoint = 0
    output_align = pid_align(angle)

    # if output_align < 0: output_align -=5
    # elif output_align >0: output_align += 5

    print(f'{output_align=}, {error=}, {angle=}')

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
        alignment_movement, alignment_error = align_robot('pololu', True)
        try:
            forward_movement = 5 / abs(alignment_error)
        except ZeroDivisionError:
             forward_movement = 5

        # print(get_angle(alignment_error))

        # if alignment_error > 0:
        #     move_right_wheel(0)
        #     move_left_wheel(abs(alignment_movement))
        # elif alignment_error < 0:
        #     move_right_wheel(abs(alignment_movement))
        #     move_left_wheel(0)
        # else:
        #     move_right_wheel(0)
        #     move_left_wheel(0)
 
        
        # move_right_wheel(3.5)
        # move_left_wheel(3.5)
        
    #     # move_left_wheel(forward_movement - alignment_movement)
    #     # move_right_wheel(forward_movement)
    #     # align_robot()
        # full_stop()
    #     # right_wheel_publisher.publish(0)
    #     # precise_tfmini_measurement()

        if global_stop_flag:
            break


    print('debug message')
    full_stop()
    #rospy.signal_shutdown('Finished Docking')
    listener_thread.join()
    print('that\'s all folks, end of the story')