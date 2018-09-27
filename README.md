# Kuka KR210 Pick and Place Project 

## Introduction  
kuka arm is an industrial robot which is 6 degree of freedom serial robot arm which has a very high precision and able to lift very heavy weights.

## Objective  
My objective is to apply forward and inverse kinematics in the form of python script using Ros to simulate on rviz and gazebo the interaction of the robot arm with the surrounding environment so that the robot arm pick the cans and put it in the box.  

## Installation steps:    
Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.  
One time Gazebo setup step: 
Check the version of gazebo installed on your system using a terminal:  
```sh
$ gazebo --version  
```
To run projects from this repository you need version 7.7.0+ If your gazebo version is not 7.7.0+, perform the update as follows:  
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
```
```sh
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
```
```sh
$ sudo apt-get update  
```
```sh
$ sudo apt-get install gazebo7  
```
Once again check if the correct version was installed:  
```sh
$ gazebo --version  
```
For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly  
If you do not have an active ROS workspace, you can create one by:  
```sh
$ mkdir -p ~/catkin_ws/src  
```
```sh
$ cd ~/catkin_ws/  
```
```sh
$ catkin_make  
```
Now that you have a workspace, clone or download this repo into the src directory of your workspace:  
```sh
$ cd ~/catkin_ws/src  
```
```sh
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git  
```
Now from a terminal window: 
```sh
$ cd ~/catkin_ws  
```
```sh
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y  
```
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts  
```
```sh
$ sudo chmod +x target_spawn.py  
```
```sh
$ sudo chmod +x IK_server.py  
```
```sh
$ sudo chmod +x safe_spawner.sh  
```
Build the project:  
```sh
$ cd ~/catkin_ws  
```
```sh
$ catkin_make  
```
Add following to your .bashrc file  
```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models  
```
```sh
source ~/catkin_ws/devel/setup.bash 
```
For demo mode make sure the demo flag is set to "true" in inverse_kinematics.launch file under /RoboND-Kinematics-Project/kuka_arm/launch  
In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the spawn_locationargument in target_description.launch file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.  
You can launch the project by  
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts  
```
```sh
$ ./safe_spawner.sh  
```
If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the demo flag described above to "false" and run your code (once the project has successfully loaded) by:  
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts  
```
```sh
$ rosrun kuka_arm IK_server.py  
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:
- Robot  

- Shelf  

- Blue cylindrical target in one of the shelves  

- Dropbox right next to the robot  
If any of these items are missing, report as an issue.  
Once all these items are confirmed, open rviz window, hit Next button.  
To view the complete demo keep hitting Next after previous action is completed successfully.  
Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.  
The demo ends when the robot arm reaches at the top of the drop location.  
There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.  
In case the demo fails, close all three terminal windows and rerun the script.


## Video of my simulation ( 9/10 ) 
[![](http://img.youtube.com/vi/7qkSZv-lOLg/0.jpg)](http://www.youtube.com/watch?v=7qkSZv-lOLg "Udacity Robot Arm")

## Forward kinematics & DH parameter table
### Extracting joint positions and orientations from URDF file.
From the urdf file **kr210.urdf** we can extract the position and orientation of the joints of the robot arm from this part of urdf file  
``` <origin xyz="X Y Z" rpy="R P Y"/> ```
and then we can get the position of the axis for the DH parameter from it
```
  <!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
  </joint>
  <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>
```
### Joint Position & Orientation Table
Following table is showing complete extracted list for all joints from base to gripper  

| O |    Joint   |     Parent     |     Child    |   x   | y |    z   | r | p | y |
|:-:|:----------:|:--------------:|:------------:|:-----:|:-:|:------:|:-:|:-:|:-:|
| 0 | fixed_base | base_footprint | base_link    | 0     | 0 | 0      | 0 | 0 | 0 |
| 1 | joint_1    | base_link      | link_1       | 0     | 0 | 0.33   | 0 | 0 | 0 |
| 2 | joint_3    | link_1         | link_2       | 0.35  | 0 | 0.42   | 0 | 0 | 0 |
| 3 | joint_4    | link_2         | link_3       | 0     | 0 | 1.25   | 0 | 0 | 0 |
| 4 | joint_5    | link_3         | link_4       | 0.96  | 0 | -0.054 | 0 | 0 | 0 |
| 5 | joint_6    | link_4         | link_5       | 0.54  | 0 | 0      | 0 | 0 | 0 |
| 6 | joint_6    | link_5         | link_6       | 0.193 | 0 | 0      | 0 | 0 | 0 |
| 7 | gripper    | link_6         | gripper_link | 0.11  | 0 | 0      | 0 | 0 | 0 |
| T | Total (m)  |                |              | 2.153 | 0 | 1.946  | 0 | 0 | 0 |

Here is a final drawing for the position of the joints and the position of the axis for the DH table:
![part1](https://user-images.githubusercontent.com/42402820/46140440-68ba9d00-c251-11e8-9f13-8a1b97e7c47f.JPG)

as you can see we obtained the parameter of DH from the drawing according to the law of DH
![capture2](https://user-images.githubusercontent.com/42402820/46140593-d1a21500-c251-11e8-81f5-75bd5041e4ae.PNG)

### Dh Table 

| Links | i | alpha(i-1) | a(i-1) | d(i)  | theta(i) |
|-------|---|------------|--------|-------|----------|
| 0->1  | 1 | 0          | 0      | 0.75  | q1       |
| 1->2  | 2 | -90        | 0.35   | 0     | q2-90    |
| 2->3  | 3 | 0          | 1.25   | 0     | q3       |
| 3->4  | 4 | -90        | -0.054 | 1.5   | q4       |
| 4->5  | 5 | 90         | 0      | 0     | q5       |
| 5->6  | 6 | -90        | 0      | 0     | q6       |
| 6->7  | 7 | 0          | 0      | 0.303 | 0        |

q(i) is our input to joint angles (theta(i)).


I will be using pytho cod for this project to obtain forward kinematics:

To start the code we need to import the following:
```
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
from sympy import symbols, cos, sin, pi, sqrt, atan2  
```
Python code to represent DH parameters table is:
```
s = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
     alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
     alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
     alpha3: -pi/2., a3: -0.054, d4:  1.50, q4:        q4,
     alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
     alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
     alpha6:      0, a6:      0, d7: 0.303, q7:         0}
```

Creating function for transformation matrices about any joint:
```
def Transform(q,d,a,alpha,s):
	T = Matrix([[cos(q)            , -sin(q)            ,  0         , a               ],
	           [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d ],
		   [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d ],
		   [0                  , 0                  ,  0         ,  1              ]])
```
Performing Transformation on joints by taking q,d,a,alpha and by default s as input and multiplying to get forward kinematics (notice i should have made def Transform(q,d,a,alpha,s=s): bec s i by default taken as input
```
	T0_1=Transform(q1,d1,a0,alpha0,s)
	T1_2=Transform(q2,d2,a1,alpha1,s)
	T2_3=Transform(q3,d3,a2,alpha2,s)
	T3_4=Transform(q4,d4,a3,alpha3,s)
	T4_5=Transform(q5,d5,a4,alpha4,s)
	T5_6=Transform(q6,d6,a5,alpha5,s)
	T6_G=Transform(q7,d7,a6,alpha6,s)
	T0_G= T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
```
Output of each joint:
```
T0_1= [cos(q1), -sin(q1), 0, 0   ]
      [sin(q1), cos(q1) , 0, 0   ]
      [      0,        0, 1, 0.75]
      [      0,        0, 0, 1]  ]

T1_2= [cos(q2 - 0.5*pi) , -sin(q2 - 0.5*pi), 0, 0.35]
      [0                , 0                , 1, 0   ]
      [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0   ]
      [0                , 0                , 0, 1   ]
      
T2_3= [cos(q3), -sin(q3), 0, 1.25]
      [sin(q3), cos(q3) , 0, 0   ]
      [0      , 0       , 1, 0   ]
      [0      , 0       , 0 , 1  ]
      
T3_4= [cos(q4) , -sin(q4), 0 , -0.054]
      [0       , 0       , 1 , 1.5   ]
      [-sin(q4), -cos(q4), 0 , 0     ]
      [0       , 0       , 0, 1      ]
      
T4_5= [cos(q5), -sin(q5), 0 , 0]
      [0      , 0       , -1, 0]
      [sin(q5), cos(q5 ), 0 , 0]
      [0      , 0       , 0 , 1]
      
T5_6= [cos(q6) , -sin(q6), 0, 0]
      [0       , 0       , 1, 0]
      [-sin(q6), -cos(q6), 0, 0]
      [0       , 0       , 0, 1]
      
T6_G= [1, 0, 0, 0    ]
      [0, 1, 0, 0    ]
      [0, 0, 1, 0.303]
      [0, 0, 0, 1    ]
     
T0_G=
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)]
[((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), ((-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), (-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), 1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)]
[(-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75]
[0, 0, 0, 1]    
``` 
Creating function for Rotation matrices :
```
R,P,Y = symbols('R P Y')
	def Rot(symb,Roll=R,Pitch=P,Yaw=Y):
		if symb == 'R':

			Rot = Matrix([
			            [   1, 0        , 0         ],
			            [   0, cos(Roll), -sin(Roll)],
			            [   0, sin(Roll),  cos(Roll)]]) 
		elif symb == 'P':
			Rot = Matrix([
			            [ cos(Pitch), 0, sin(Pitch)],
			            [ 0         , 1,          0],
			            [-sin(Pitch), 0, cos(Pitch)]])  
		elif symb == 'Y':
			Rot = Matrix([
			            [cos(Yaw), -sin(Yaw), 0],
			            [sin(Yaw),  cos(Yaw), 0],
			            [      0,          0, 1]])  

		return Rot
```

Accounting for Orientation Difference Between definition of Gripper Link_7 in URDF(r=0,p=0,y=0) versus DH so we per rotate around y then around z axis and perform externic rotation:
```
    Rot_x = Rot('R')
    Rot_y = Rot('P')
    Rot_z = Rot('Y')
    Rot_F = Rot_z.subs(Y,radians(180))*Rot_Y.subs(P,radians(-90))
    Rot_E = Rot_z*Rot_y*Rot_x
    Rot_EE = Rot_E * Rot_F
```


Inverse Kinematics 
Choosing Joint 5 as Wrist
### theta 1,2,3
theta 1 is the easiest to be obtained it is just atan2(y, x)
``` theta1 = atan2(WC_y , WC_x) ```
![part3](https://user-images.githubusercontent.com/42402820/46146622-84c73a00-c263-11e8-90a8-02ddaa5aa1e7.JPG)

### theta 2,3 
By applying the cosine rule to obtain angle 3 first, then calculating angle 2. The figure below explains how thetas is obtained  
![coslaw](https://user-images.githubusercontent.com/42402820/46146880-31092080-c264-11e8-929a-884fd2af1fad.png)
![part2](https://user-images.githubusercontent.com/42402820/46146179-70367200-c262-11e8-9caa-87c2090b2fb4.JPG)
Forming the Equation to obtain theta 2,3
```
	La = 1.502 
	Lc = 1.25
	a1 = 0.35
	d1 = 0.75
	Lxy= sqrt(pow(WC_x, 2.) + pow(WC_y, 2.) ) - a1
	Lz = WC_z - d1
	Lb = sqrt(pow(Lxy, 2.) + pow(Lz, 2.))
    
    a_ang = acos( ( pow(Lb, 2.) + pow(Lc, 2.) - pow(La, 2.)) / (2. * Lb * Lc) )
    b_ang = acos( ( pow(La, 2.) + pow(Lc, 2.) - pow(Lb, 2.)) / (2. * La * Lc) )
    c_ang = acos( ( pow(La, 2.) + pow(Lb, 2.) - pow(Lc, 2.)) / (2. * La * Lb) )
    theta1 = atan2(WC_y , WC_x)
    theta2 = 90. - a_ang - atan2(Lz/Lxy)
    theta3 = 90. - Lb - atan2(0.054/1.5)
```
Evaluating Transformation from 0 to 3
```
   R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={theta1: theta1,theta2: theta2,theta3: theta3})[0:3, 0:3]
```

### Calculating theta 4,5,6
we first solved for theta 1,2,3 geometically so now to get theta 4,5,6 we can mulyiply the transpose of the rotation matrix from 1 to 3 (R0_3 as we evaluated it from the previous step) by the Rotation of the end effector to obtain the rotation from 3 to 6 (this is according to laws of matrices)
```
    R3_6 = R0_3.T * R_EE
```
So now we can slove for the thets as we evaluated the matrix and we have the orginal matrix so by equating both of them we will be able to find the remaining thetas according to this laws

![qq](https://user-images.githubusercontent.com/42402820/46147411-bc36e600-c265-11e8-98ee-a19b9fc9e07c.PNG)
![qqq](https://user-images.githubusercontent.com/42402820/46147412-bc36e600-c265-11e8-8894-759ad7da5770.PNG)

and implementing it in the form of code we obtain the remaining thetas and that is done by finding theta 5 an d finding the best choice for theta 4, 6
```
    theta5 = atan2(sqrt(pow(R3_6[0,2], 2) + pow(R3_6[2,2], 2)), R3_6[1,2])
	if sin(theta5) < 0:
	    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
	    theta6 = atan2(R3_6[1,1], -R3_6[1,0])
	else:
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

## Improvments
- Finding solution for failing to pick the target sometimes and i was able to increase the probability of picking the target by making a change in trajectory_sampler.cpp in line by making ``` target_reach.position.x = target_x - 0.2; ``` to ``` target_reach.position.x = target_x - 0.15; ``` this will make the girpper go closer to the target
- Finding solution for Discintinous that happen during simulation and that is done by calculating theta 5 and then finding the best choice for theta 4,6
- Another challenge is to avoid using symbolic proccessed inside the loop and this is done by doing all symbol simulation before entering the function ``` handle_calculate_IK(req) ```


    
