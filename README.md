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

Each wheel has to be controlled individually. 

### Alignemnt

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

