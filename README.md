![license][]

# DISCLAIMER

Work is still in progress! 


# AGV-Autonomous-Docking
Algorithm for autonomous docking of AGV platform.

## Problem

Alrogithm that allows robot with differential drive to dock to two types of platforms:

Types of docking stations:
- Straight wall
- Walls in a shape of a letter "L"

Docking means positioning the robot with proper alignemnt and distance to wall or walls. 
 
## Solution

Solution was divided into three parts.

### Wheel Control

Each wheel has to be controlled individually using PID regulator. PID helps to adequate PWM signals that achieves chooses RPM in give conditions.   

Input - RPM
Output - PWM

|   | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
|  Right wheel |  6 | 5  | 0.1  |
|  Left wheel | 5  | 5  |  0.1 |

### Alignemnt

Next part of the algorithm is to align robot against the wall. To do that, another PID controller is used. It takes angle of required alignment as an input and as an output returns speed (RPM) of wheels that allows robot to rotate. 

<p align="center">
  <img src="images/aligned.png" width="500" />
</p>

<p align="center">
  <img src="images/align_1.png" width="500" />
</p>

<p align="center">
  <img src="images/align_2.png" width="500" />
</p>


#### Angle calculation

Angle of the robot against the wall. 

### Distance Control

#### Distance calculation

Calculates distance of a point of the robot, which is placed in the center of the wheels axle. It is the same point around which AGV platform rotates. 

## Techonologies

- Python
- ROS
- Nvidia Xavier

## Sensors

- Lidar
- ToF

[license]:https://img.shields.io/github/license/szymciem8/AGV-Autonomous-Docking
