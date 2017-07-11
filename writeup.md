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

[image1]: ./misc_images/robond-kinematics.jpg
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

The file kr210.urdf.xacro and the graph shown above, are the sources to obtain the DH table, as follows:

* alpha<sub>0</sub> is zero becausen z<sub>0</sub> and z<sub>1</sub> are parallel.
* a<sub>0</sub> is zero because there is no displacement between z<sub>0</sub> and z<sub>1</sub>.
* d<sub>1</sub> is the diplacement between x<sub>0</sub> and x<sub>1</sub> along z<sub>1</sub>, which in the xacro file is the z position of joint_1 w.r.t the base (0.33), plus the z position of joint_2 w.r.t joint_1 (0.42).

* alpha<sub>1</sub> is the angle between z<sub>1</sub> and z<sub>2</sub> measured about x<sub>1</sub> (-pi/2).
* a<sub>1</sub> is the displacement between z<sub>1</sub> and z<sub>2</sub> along x<sub>1</sub>, which in the xacro file is the x position of joint_2 with respect to joint_1 (0.35).
* d<sub>2</sub> is zero because there is no displacement between x<sub>1</sub> and x<sub>2</sub>.

* alpha<sub>2</sub> is zero because z<sub>2</sub> and z<sub>3</sub> are parallel.
* a<sub>2</sub> is the displacement between z<sub>2</sub> and z<sub>3</sub> along x<sub>2</sub>, which in the xacro file is the z position of joint_3 w.r.t joint_2 (1.25).
* d<sub>3</sub> is zero because there is no displacement between x<sub>2</sub> and x<sub>3</sub>

* alpha<sub>3</sub> is the angle between z<sub>3</sub> and z<sub>4</sub> measured about x<sub>3</sub> (-pi/2).
* a<sub>3</sub> is the displacement between z<sub>3</sub> and z<sub>4</sub> along x<sub>3</sub>, which in the xacro file is the z position of joint_4 w.r.t joint_3 (-0.054)
* d<sub>4</sub> is the displacement between x<sub>3</sub> and x<sub>4</sub> along z<sub>4</sub>, which in the xacro file is the x position of joint_4 w.r.t joint_3 (0.96) plus the x position of joint_5 w.r.t joint_4 (0.54)

* alpha<sub>4</sub> is the angle between z<sub>4</sub> and z<sub>5</sub> measured about x<sub>4</sub> (pi/2).
* a<sub>4</sub> is zero because there is no displacement between z<sub>4</sub> and z<sub>5</sub>.
* d<sub>5</sub> is zero because there is no displacement between x<sub>4</sub> and x<sub>5</sub>.

* alpha<sub>5</sub> is the angle between z<sub>5</sub> and z<sub>6</sub> measured about x<sub>5</sub> (-pi/2).
* a<sub>5</sub> is zero because there is no displacement between z<sub>5</sub> and z<sub>6</sub>.
* d<sub>6</sub> is zero because there is no displacement between x<sub>5</sub> and x<sub>6</sub>.

* alpha<sub>6</sub> is zero because z<sub>6</sub> and z<sub>7</sub>, a.k.a z<sub>G</sub>, are parallel.
* a<sub>6</sub> is zero because there is no displacement between z<sub>6</sub> and z<sub>7</sub>.
* d<sub>7</sub> (a.k.a. d<sub>G</sub>) is the displacement between x<sub>6</sub> and x<sub>7</sub> (a.k.a. x<sub>G</sub>) along z<sub>7</sub>, which in the xacro file is the x position of joint_6 w.r.t. joint_5 (0.193) plus the x position of the gripper_joint w.r.t joint_6 (0.11)

The obtained DH table is as follows:

<sup>i-1</sup>T<sub>i</sub> | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | theta<sub>i</sub>
--- | --- | --- | --- | ---
<sup>0</sup>T<sub>1</sub> | alpha<sub>0</sub> = 0 | a<sub>0</sub> = 0 | d<sub>1</sub> = 0.75 | theta<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | alpha<sub>1</sub> = -pi/2 | a<sub>1</sub> = 0.35 | d<sub>2</sub> = 0 | theta<sub>2</sub> = theta<sub>2</sub>-pi/2
<sup>2</sup>T<sub>3</sub> | alpha<sub>2</sub> = 0 | a<sub>2</sub> = 1.25 | d<sub>3</sub> = 0 | theta<sub>3</sub>
<sup>3</sup>T<sub>4</sub> | alpha<sub>3</sub> = -pi/2 | a<sub>3</sub> = -0.054 | d<sub>4</sub> = 1.50 | theta<sub>4</sub>
<sup>4</sup>T<sub>5</sub> | alpha<sub>4</sub> = pi/2 | a<sub>4</sub> = 0 | d<sub>5</sub> = 0 | theta<sub>5</sub>
<sup>5</sup>T<sub>6</sub> | alpha<sub>5</sub> = -pi/2 | a<sub>5</sub> = 0 | d<sub>6</sub> = 0 | theta<sub>6</sub>
<sup>6</sup>T<sub>7</sub> | alpha<sub>6</sub> = 0 | a<sub>6</sub> = 0 | d<sub>7</sub> = 0.303 | theta<sub>7</sub> = 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The intersecion between x<sub>i</sub> and z<sub>i</sub> is the origin of the reference frame i. Let this origin be O<sub>i</sub>. The transformation from O<sub>i-1</sub> to O<sub>i</sub>, denoted by <sup>i-1</sup>T<sub>i</sub>, is the multiplication of the transformation matrices R<sub>x</sub>(alpha<sub>i-1</sub>)D<sub>x</sub>(a<sub>i-1</sub>)R<sub>z</sub>(theta<sub>i</sub>)D<sub>z</sub>(d<sub>i</sub>), which give the following matrix:

ABC | BCD | CDE | DEF
--- | --- | --- | ---
cos(theta<sub>i</sub>) | -sin(theta<sub>i</sub>) | 0 | a<sub>i-1</sub>
sin(theta<sub>i</sub>)cos(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>)cos(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>)d<sub>i</sub>
sin(theta<sub>i</sub>)sin(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>)sin(alpha<sub>i-1</sub>) | cos(alpha<sub>i-1</sub>) | cos(alpha<sub>i-1</sub>)d<sub>i</sub>
0 | 0 | 0 | 1

The homogeneous transformation matrix from base the end-effector is obtained by (<sup>0</sup>T<sub>ee</sub>) = (<sup>0</sup>T<sub>1</sub>)(<sup>1</sup>T<sub>2</sub>)(<sup>2</sup>T<sub>3</sub>)(<sup>3</sup>T<sub>4</sub>)(<sup>4</sup>T<sub>5</sub>)(<sup>5</sup>T<sub>6</sub>)(<sup>6</sup>T<sub>7</sub>)(<sup>7</sup>T<sub>ee</sub>), where <sup>7</sup>T<sub>ee</sub> is a correction matrix needed because the urdf specification uses a diferent reference frame at the end-effector than DH. This correction matrix is obtained by rotating 180 degrees around z, and -90 degrees around y. The x and z urdf reference frame coordinates are depicted in the image previously shown as x<sub>w</sub> and z<sub>w</sub>, respectively.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

<sup>i-1</sup>T<sub>i</sub> | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d | theta
--- | --- | --- | --- | ---
bla | ble | bli | blo | blu<br>

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


