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

xavier_setup = 'export ROS_MASTER_URI=http://192.168.1.101:11311 && export ROS_IP=192.168.1.112'

right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

rospy.init_node('robot_controller', anonymous=True)

#W miarę OK
Kp = 14
Ki = 7
Kd = 0.3

#Setpoint [rad/s]

# RIGHT WHEEL PID
pid_right = PID(Kp, Ki, Kd, setpoint=0)
pid_right.sample_time = 0.01
pid_right.output_limits = (-1000, 1000)

# LEFT WHEEL PID
pid_left = PID(Kp, Ki, Kd, setpoint=0)
pid_left.sample_time = 0.01
pid_left.output_limits = (-1000, 1000)

# ALIGNMENT PID
# Nastawa początkowa P=0.4, I=0, D=0

T_kr = 1/0.625
Kr = 1.75

pid_align = PID(1.5, 0.2, 0.008) # było 0.5, 0.1, 0.0
pid_align.sample_time = 0.01
pid_align.setpoint=0
pid_align.output_limits = (-15, 15)

right_wheel_speed = 0
left_wheel_speed = 0

speed_values = {}
speed_values['right'] = []
speed_values['left'] = []

rospy.init_node('robot_controller', anonymous=True)

tfmini_measurements = [0, 0, 0, 0]
pololu_measurements = [0, 0, 0, 0]

frame_size = 10
average_tfmini = [[0 for i in range(frame_size)] for _ in range(4)]

def precise_tfmini_measurement():
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
    p1.wait()

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
    print(right_wheel_speed, left_wheel_speed)

def listener():
    rospy.Subscriber('/joint_states', JointState, callback_joint_state)
    for i in range(4):
        rospy.Subscriber('/mega_driver/pololu/scan_'+str(i), Range, callback_pololu, callback_args=i)
        rospy.Subscriber('/mega_driver/tfmini/scan_'+str(i), Range, callback_tfmini, callback_args=i)

    rospy.spin()

def move_right_wheel(speed):
    global pid_right, right_wheel_speed

    pid_right.setpoint=speed
    output = pid_right(right_wheel_speed)

    if output > 0: output += 300
    elif output < 0: output -= 300

    # print(output)
    right_wheel_publisher.publish(output)

def move_left_wheel(speed):
    global pid_left, left_wheel_speed 

    pid_left.setpoint=speed
    output = pid_left(left_wheel_speed)
    left_wheel_publisher.publish(output)

def full_stop():
    right_wheel_publisher.publish(0)
    left_wheel_publisher.publish(0)

def align_robot():
    global pid_align
    
    # LF pololu_scan_0
    # LR pololu_scan_0

    error_pololu = pololu_measurements[3] - pololu_measurements[2] 
    error_tfmini = tfmini_measurements[3] - tfmini_measurements[2]

    precise_tfmini = precise_tfmini_measurement()
    error_tfmini = precise_tfmini[3] - precise_tfmini[2]
    
    # print('tf2', tfmini_measurements[2], end='\t')
    # print('tf3', tfmini_measurements[3], end='\t')
    # print('tf_error', error_tfmini)
    # if error_pololu <= -7: error_pololu=0
    # elif error_pololu >= 7: error_pololu=0

    pid_align.setpoint=0

    if -11 <= error_tfmini <= 11: error_tfmini = 0

    print('tf_error', error_tfmini)

    # print(error_tfmini, end='\t')
    output_align = pid_align(error_tfmini)

    # if output_align > 0: output_align += 10
    # elif output_align <0: output_align -= 10

    move_right_wheel(output_align)

    # print('error pololu', error_pololu)
    # print('error tfmini', error_tfmini)
    # print('pid output', output_align) 


if __name__=='__main__':

    #Use right_velocity and left_velocity instead of diff_drive
    #Disconnect built in PID controller
    controller_manager_setup()

    #Start listener thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    try:
        while True:
            align_robot()
            #move_right_wheel(10)
            #move_right_wheel(20)
            # time.sleep(0.01)
            #full_stop()
            #precise_tfmini_measurement()

    # time.sleep(5)
    
    except KeyboardInterrupt:
        full_stop()
        listener_thread.join()