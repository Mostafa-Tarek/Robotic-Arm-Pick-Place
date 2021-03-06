## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

# DH Parameters
'''
DH Table
Link    a       alpha   d       theta
1       a1      90      d1      *
2       a2      0       0       *
3       -a3     90      d3      *
4       0       -90     0       *
5       0       90      0       *
6       0       0       d6      *
'''


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 
The first containing joints 1, 2, and 3, as well as the base link, arm 1 and arm 2. This 3DOF (RRR) setup can be considered as the elbow section of the arm. The inverse joint angle calculation of this elbow can then be derived by using basic trigonometry and the law of cosine.

overview

Joint 1: Angle from position in the x-y plane
Joint 2: Law of cosine in the x, y, z planes and perpendicular to the z-plane.
Joint 3: The difference in the law of cosine angle a continuation of link 2.
The remaining joints make up the wrist section of the arm, and joint angles have been derived using trigonometric laws and current robot orientation and Position.

overview

Joint 4: Tangent to a centre point in the centre of the pickup shelve location. I.e., joint three will rotate around the centre point as if it was the tangent of the circle with a zero angle when at the bottom, pi/2 rotation angle on the left side, pi rotation when verticle and 3*pi/2 on the right side.
Joint 5: Angle is derived from the most direct line to the target pointing the gripper at the target.
Joint 6: negative of Joint 4's angle.
Below are the calculations of all 6 joint angles.



![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


