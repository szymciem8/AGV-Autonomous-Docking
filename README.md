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

Docking means positioning the robot with proper alignemnt and distance to the wall or walls. 
 
## Solution

Solution was divided into three parts:
- Wheel control
- Alignment
- Distance control

All those parts are based on using PID controlers for different actions.

### Wheel Control

Each wheel has to be controlled individually using PID regulator. PID helps to generate adequate PWM signal that achieves choosen RPM in given conditions.   

Input - RPM
Output - PWM

|Settings   | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
|  Right wheel |  6 | 5  | 0.1  |
|  Left wheel | 5  | 5  |  0.1 |

### Alignemnt

Next part of the algorithm is to align robot against the wall. To do that, another PID controller is used. It takes angle of required alignment as an input and as an output returns speed (RPM) of wheels that allows robot to rotate. 

Input - Angle
Output - RPM

| Settings  | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
|  Wheel RPM |  3 | 0.5  | 0.008  |

<p align="center">
  <img src="images/aligned.png" width="500" />
</p>


With help of differtial drive, robot can be rotated by spinning wheels in the opposite directions. RPM can be a positive or negative based on the direction in which the robot is suposed to turn. 

Black squares on the AGV platform are distance sensor, either Lidar or ToF. 

<p align="center">
  <img src="images/align_1.png" width="500" />
</p>

<p align="center">
  <img src="images/align_2.png" width="500" />
</p>

The advatage of this algorithm is that the robot can be aligned in any angle, something between -0.3 and 0.3 radians. 

#### Angle calculation

Angle of the robot against the wall. 

Error is calculated as a difference betweeen L1 and L2. 

``
error = L1 - L2
``

```
 def get_angle(self, error):
     D = 195
     return math.asin(error/math.sqrt(D**2+error**2)) 
```

### Distance Control

As stated before, because robot can achive any angle alignemnt, it can be steered away or to the wall. This way, AGV can achieve any distance from the wall. 

To achive that, another PID regulator was used. This time it calculates the angle that the robot is supposed to achieve based on the difference between current distance and selected distance. 

Input - Difference between distances
Output - Angle

| Settings  | Kp  | Ki  | Kd  | 
|:-:|:-:|:-:|:-:|
| Angle |  0.002 | 0.00002  | 0.00002  |

#### Distance calculation

<p align="center">
  <img src="images/measure_distance.png" width="500" />
</p>

```
 def get_distance_from_wall(self, l1, l2):
     # In mm
     X0 = 150
     Y0 = 87

     # d/l1 = cos(alfa) -> d = li

     angle = self.get_angle(l2-l1)

     d = l1 * math.cos(angle)

     return d + math.cos(angle) * X0 - math.sin(angle) * Y0  
```

Calculates distance of a point of the robot, which is placed in the center of the wheels axle. It is the same point around which AGV platform rotates. 

## Machine Learning -- IN PROGRESS!

System will be equiped with machine learning model that will state if docking is possible based on previous attemps.

div{

border: 5px solid blue;

width: 300px;

height: 300px;

background-color: red;

}

<p align="center">
  <img src="images/from_logs/ride_0_base_speed_2.8_without_rosbag.png" width="1000" />
</p>

<p align="center">
  <img src="images/from_logs/ride_6_base_speed_2.5_without_rosbag.png" width="1000" />
</p>

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
