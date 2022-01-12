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

Solution was divided into three parts:
- Wheel control
- Alignment
- Distance control

All those parts are based on using PID controlers, with different tasks.

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

Input - Angle
Output - RPM

|   | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
|  Wheel RPM |  3 | 0.5  | 0.008  |

<p align="center">
  <img src="images/aligned.png" width="500" />
</p>

RPM can be a positive or negative number based on the direction in which the robot is suposed to turn. 

<p align="center">
  <img src="images/align_1.png" width="500" />
</p>

<p align="center">
  <img src="images/align_2.png" width="500" />
</p>

The advatage of this algorithm is that the robot can be aligned in any angle, something between -0.3 rad and 0.3 rad. 

#### Angle calculation

Angle of the robot against the wall. 

``

 def get_angle(self, error):
     D = 195
     return math.asin(error/math.sqrt(D**2+error**2))
     
``

### Distance Control

As stated before, because robot can achive any angle alignemnt, it can be steered away or to the wall. This way, AGV can achieve any distance from the wall. 

To achive that, another PID regulator was used. This time it calculates the angle that the robot is supposed to achieve based on the difference between current distance and selected distance. 

Input - Difference between distances
Output - Angle

|   | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
| Angle |  0.002 | 0.00002  | 0.00002  |

#### Distance calculation

``

 def get_distance_from_wall(self, l1, l2):
     # In mm
     X0 = 150
     Y0 = 87

     # d/l1 = cos(alfa) -> d = li

     angle = self.get_angle(l2-l1)

     d = l1 * math.cos(angle)

     return d + math.cos(angle) * X0 - math.sin(angle) * Y0
     
``

Calculates distance of a point of the robot, which is placed in the center of the wheels axle. It is the same point around which AGV platform rotates. 

## Summary

To sum up, the algorithm consits of chainged PID controllers that control three properties: wheel speed, alignment (angle) and distance. By controlling those values, ....

## Techonologies

- Python
- ROS
- Nvidia Xavier

## Sensors

- Lidar
- ToF

[license]:https://img.shields.io/github/license/szymciem8/AGV-Autonomous-Docking
