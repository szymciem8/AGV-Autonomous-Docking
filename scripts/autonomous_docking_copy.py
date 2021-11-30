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


right_wheel_publisher = rospy.Publisher('right_velocity/command', Float64, queue_size=10)
left_wheel_publisher = rospy.Publisher('left_velocity/command', Float64, queue_size=10)

rospy.init_node('robot_controller', anonymous=True)

#W miarę OK
Kp = 7
Ki = 7
Kd = 0.03

#Setpoint [rad/s]
pid_right = PID(Kp, Ki, Kd, setpoint=0)
pid_right.sample_time = 0.01

pid_right.output_limits = (0, 1000)

pid_left = PID(Kp, Ki, Kd, setpoint=0)
pid_left.sample_time = 0.01
pid_left.output_limits = (0, 1000)

right_wheel_speed = 0
left_wheel_speed = 0


speed_values = {}
speed_values['right'] = []
speed_values['left'] = []

rospy.init_node('robot_controller', anonymous=True)

# PLOT
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'r-')

def init():
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    return ln,

def update(frame):
    global xdata, ydata

    ln.set_data(xdata, ydata)
    return ln,

def controller_manager_setup():
    command = 'xavier_setup'
    command += ' && rosservice call /robot_driver/motor_contoller/turn_off_PID True'
    command += ' && rosrun controller_manager controller_manager stop diff_drive'
    command += ' && rosrun controller_manager controller_manager unload diff_drive'
    command += ' && rosrun controller_manager controller_manager spawn right_velocity'
    command += ' && rosrun controller_manager controller_manager spawn left_velocity'
    p1=Popen(['/bin/bash', '-i', '-c', command])
    p1.wait()


def callback_joint_state(msg):
    global right_wheel_speed, left_wheel_speed
    right_wheel_speed = msg.velocity[1]
    left_wheel_speed = msg.velocity[0]
    print(right_wheel_speed, left_wheel_speed)

def callback_stm_imu(msg):
    global angular_velocity
    angular_velocity = msg.angular_velocity.z
    print

def callback_pololu(msg, i):
    print(i)

def callback_tfmini(msg, i):
    print(i)


def listener():
    # rospy.Subscriber('/joint_states', JointState, callback_joint_state)
    # rospy.Subscriber('/stm_imu', Imu, callback_stm_imu)  

    for i in range(4):
        rospy.Subscriber('/mega_driver/pololu/scan_'+str(i), Range, callback_pololu, callback_args=i)
        rospy.Subscriber('/mega_driver/pololu/scan_'+str(i), Range, callback_pololu, callback_args=i)

    rospy.spin()

def move_right_wheel(speed):
    global pid_right, right_wheel_speed

    pid_right.setpoint=speed
    output = pid_right(right_wheel_speed)
    right_wheel_publisher.publish(output)

def move_left_wheel(speed):
    global pid_left, left_wheel_speed 

    pid_left.setpoint=speed
    output = pid_left(left_wheel_speed)
    left_wheel_publisher.publish(output)


def move_forward(speed=10, distance=100):
    global right_wheel_speed, left_wheel_speed
    global speed_values

    current_distance=0
    t0 = rospy.Time.now().to_sec()
    while current_distance < distance:
        move_right_wheel(speed)
        move_left_wheel(speed)
        print(current_distance)
        avg_speed = (right_wheel_speed+left_wheel_speed)/2

        speed_values['right'].append(right_wheel_speed)
        speed_values['left'].append(left_wheel_speed)

        t1 = rospy.Time.now().to_sec()
        current_distance = (t1-t0) * avg_speed

    full_stop()


def rotate(angle):
    measured_angle=0

    while measured_angle < angle:
        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        # measured_angle += (t1-t0) * angular_velocity
        measured_angle += (t1-t0) * 1
        print(measured_angle)

        if t1-t0 > 10:
            break

def full_stop():
    right_wheel_publisher.publish(0)
    left_wheel_publisher.publish(0)

def move(velocity=25, distance=200):
    global right_wheel_publisher, left_wheel_publisher  
    global right_wheel_speed, left_wheel_speed
    global pid_right

    values = []

    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    right_wheel_publisher.publish(0)
    current_distance=0
    t0 = rospy.Time.now().to_sec()

    while current_distance < distance:
        #Setpoint value using PID
        print(right_wheel_speed)
        values.append(right_wheel_speed)
        pid_right.setpoint=velocity
        
        #Controll motor with PID
        output = pid_right(right_wheel_speed)

        #Set RW speed to the one given by PID
        right_wheel_publisher.publish(output)

        t1 = rospy.Time.now().to_sec()
        current_distance = (t1-t0)*velocity
        print(current_distance)

    right_wheel_publisher.publish(0)
    left_wheel_publisher.publish(0)
    listener_thread.join()

if __name__=='__main__':

    #Use right_velocity and left_velocity instead of diff_drive
    #Disconnect built in PID controller
    # controller_manager_setup()

    #Start listener thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    # move_forward(10, 50)

    # rotate(90)

    # plt.plot(speed_values['right'])
    # plt.plot(speed_values['left'])
    # plt.show()

    time.sleep(5)

    listener_thread.join()