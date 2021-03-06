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

The file kr210.urdf.xacro and the graph shown above are the sources to obtain the DH table, as follows:

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

The intersecion between x<sub>i</sub> and z<sub>i</sub> is the origin of the reference frame i. Let this origin be O<sub>i</sub>. The transformation from O<sub>i-1</sub> to O<sub>i</sub>, denoted by <sup>i-1</sup>T<sub>i</sub>, is the multiplication of the transformation matrices R<sub>x</sub>(alpha<sub>i-1</sub>)D<sub>x</sub>(a<sub>i-1</sub>)R<sub>z</sub>(theta<sub>i</sub>)D<sub>z</sub>(d<sub>i</sub>), where R<sub>i</sub> is the transformation matrix expressing the rotation around the i axis, and D<sub>i</sub> is the transformation matrix expressing the translation along the i axis. This multiplication gives the following matrix:

NOT | A | MATRIX | ROW
--- | --- | --- | ---
cos(theta<sub>i</sub>) | -sin(theta<sub>i</sub>) | 0 | a<sub>i-1</sub>
sin(theta<sub>i</sub>)cos(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>)cos(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>) | -sin(alpha<sub>i-1</sub>)d<sub>i</sub>
sin(theta<sub>i</sub>)sin(alpha<sub>i-1</sub>) | cos(theta<sub>i</sub>)sin(alpha<sub>i-1</sub>) | cos(alpha<sub>i-1</sub>) | cos(alpha<sub>i-1</sub>)d<sub>i</sub>
0 | 0 | 0 | 1

The homogeneous transformation matrix from base the end-effector is obtained by <sup>0</sup>T<sub>ee</sub> = <sup>0</sup>T<sub>1</sub>.<sup>1</sup>T<sub>2</sub>.<sup>2</sup>T<sub>3</sub>.<sup>3</sup>T<sub>4</sub>.<sup>4</sup>T<sub>5</sub>.<sup>5</sup>T<sub>6</sub>.<sup>6</sup>T<sub>7</sub>.<sup>7</sup>T<sub>ee</sub>, where <sup>7</sup>T<sub>ee</sub> is a correction matrix needed because the transformation at the end-effector is in urdf coordinates, which uses a different specification than the DH convention. This correction matrix is obtained by rotating 180 degrees around z, and -90 degrees around y. The x and z coordinates in urdf are depicted in the image previously shown as x<sub>w</sub> and z<sub>w</sub>, respectively.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

We have a system with 6 unknowns, i.e., theta<sub>1</sub>, theta<sub>2</sub>, theta<sub>3</sub>, theta<sub>4</sub>, theta<sub>5</sub>, theta<sub>6</sub>. The position and rotation of the end-effector are known. We could use these position and rotation to try to solve for the 6 unknowns. But instead, an easier way to approach this problem is to use the fact that between the origins O<sub>4</sub>, O<sub>5</sub> and O<sub>6</sub>, there are no displacements, i.e., a<sub>4</sub> = a<sub>5</sub> = d<sub>5</sub> = d<sub>6</sub> = 0, and that the displacement d<sub>7</sub> from O<sub>6</sub> to O<sub>7</sub> is along the x axis of the end-effector in urdf coordinates. So, we could subtract d<sub>7</sub> from the end-effector position, using the orthonormal vector representing the projection of the x axis of the end-effector reference frame onto the world coordinates, which are the coordinates of O<sub>0</sub>, and obtain the position of O<sub>6</sub>, which is the postion of O<sub>4</sub>, both in world coordinates. The position of O<sub>4</sub> only depends on theta<sub>1</sub>, theta<sub>2</sub> and theta<sub>3</sub>, so we equate the translation component from <sup>0</sup>T<sub>4</sub> = <sup>0</sup>T<sub>1</sub>.<sup>1</sup>T<sub>2</sub>.<sup>2</sup>T<sub>3</sub>.<sup>3</sup>T<sub>4</sub>, which is expressed in terms of theta<sub>1</sub>, theta<sub>2</sub> and theta<sub>3</sub>, with the position obtained by subtracting d<sub>7</sub> from the end-effector position.

**I** = (I<sub>x</sub>,I<sub>y</sub>,I<sub>z</sub>) = Orthonormal vector representing the projection of the x axis of the end-effector reference frame onto the world coordinates.

**p** = (p<sub>x</sub>,p<sub>y</sub>,p<sub>z</sub>) = End-effector position.

**w** = (w<sub>x</sub>,w<sub>y</sub>,w<sub>z</sub>) = O<sub>4</sub> position, a.k.a. wrist center

**t** = (t<sub>x</sub>,t<sub>y</sub>,t<sub>z</sub>) = Translation component of transformation from O<sub>0</sub> to O<sub>4</sub>, i.e., <sup>0</sup>T<sub>4</sub>

**w** = **p** - **I**.d<sub>7</sub>

t<sub>x</sub> = cos(theta<sub>1</sub>)(a<sub>2</sub>.sin(theta<sub>2</sub>) - a<sub>3</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>) + d<sub>4</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) + a<sub>1</sub>)

t<sub>y</sub> = sin(theta<sub>1</sub>)(a<sub>2</sub>.sin(theta<sub>2</sub>) - a<sub>3</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>) + d<sub>4</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) + a<sub>1</sub>)

t<sub>z</sub> = -d<sub>4</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>) + a<sub>2</sub>.cos(theta<sub>2</sub>) - a<sub>3</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) + d<sub>1</sub>

Equating **w** and **t**, theta<sub>1</sub>, theta<sub>2</sub> and theta<sub>3</sub> can be obtained, as follows:

theta<sub>1</sub> = atan2(w<sub>y</sub>, w<sub>x</sub>)

Knowing theta<sub>1</sub>, we can calculate:

(w<sub>x</sub>/cos(theta<sub>1</sub>) - a<sub>1</sub>)<sup>2</sup> = a<sub>2</sub><sup>2</sup>.sin<sup>2</sup>(theta<sub>2</sub>) + a<sub>3</sub><sup>2</sup>.sin<sup>2</sup>(theta<sub>2</sub> + theta<sub>3</sub>) - 2.a<sub>2</sub>.sin(theta<sub>2</sub>).a<sub>3</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>) + 2.a<sub>2</sub>.sin(theta<sub>2</sub>).d<sub>4</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) - 2.a<sub>3</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>).d<sub>4</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) + d<sub>4</sub><sup>2</sup>.cos<sup>2</sup>(theta<sub>2</sub> + theta<sub>3</sub>)

(w<sub>z</sub> - d<sub>1</sub>)<sup>2</sup> = d<sub>4</sub><sup>2</sup>.sin<sup>2</sup>(theta<sub>2</sub> + theta<sub>3</sub>) - 2.d<sub>4</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>).a<sub>2</sub>.cos(theta<sub>2</sub>) + a<sub>2</sub><sup>2</sup>.cos<sup>2</sup>(theta<sub>2</sub>) + 2.d<sub>4</sub>.sin(theta<sub>2</sub> + theta<sub>3</sub>).a<sub>3</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) - 2.a<sub>2</sub>.cos(theta<sub>2</sub>).a<sub>3</sub>.cos(theta<sub>2</sub> + theta<sub>3</sub>) + a<sub>3</sub><sup>2</sup>.cos<sup>2</sup>(theta<sub>2</sub> + theta<sub>3</sub>)

Adding these last two equations and re-arranging, we obtain:

(d<sub>4</sub><sup>2</sup> + a<sub>3</sub><sup>2</sup> + a<sub>2</sub><sup>2</sup> - (w<sub>x</sub>/cos(theta<sub>1</sub>) - a<sub>1</sub>)<sup>2</sup> - (w<sub>z</sub> - d<sub>1</sub>)<sup>2</sup>) / (2.a<sub>2</sub>) = a<sub>3</sub>.cos(theta<sub>3</sub>) - d<sub>4</sub>.sin(theta<sub>3</sub>) = k

where k is a constant.

using the substitution:

sin(theta<sub>3</sub>) = 2.u / (1 + u<sup>2</sup>)

cos(theta<sub>3</sub>) = (1 - u<sup>2</sup>) / (1 + u<sup>2</sup>)

where u = tan(theta<sub>3</sub> / 2)

we obtain a quadratic equation with two solutions for u:

u<sub>1</sub> = (d<sub>4</sub> + sqrt(d<sub>4</sub><sup>2</sup> - k<sup>2</sup> + a<sub>3</sub><sup>2</sup>)) / (k + a<sub>3</sub>)

u<sub>2</sub> = (d<sub>4</sub> - sqrt(d<sub>4</sub><sup>2</sup> - k<sup>2</sup> + a<sub>3</sub><sup>2</sup>)) / (k + a<sub>3</sub>)

Given the configuration space, theta<sub>3</sub> tends to be small, hence we choose u<sub>2</sub>.

To obtain theta<sub>2</sub>, we multiply w<sub>x</sub>/cos(theta<sub>1</sub>) - a<sub>1</sub> by sin(theta<sub>2</sub>), and w<sub>z</sub> - d<sub>1</sub> by cos(theta<sub>2</sub>) and add them, obtaining:

(w<sub>x</sub>/cos(theta<sub>1</sub>) - a<sub>1</sub>).sin(theta<sub>2</sub>) + (w<sub>z</sub> - d<sub>1</sub>).cos(theta<sub>2</sub>) = a<sub>2</sub> - k

again, using the substitution:

sin(theta<sub>2</sub>) = 2.v / (1 + v<sup>2</sup>)

cos(theta<sub>2</sub>) = (1 - v<sup>2</sup>) / (1 + v<sup>2</sup>)

where v = tan(theta<sub>2</sub> / 2)

we obtain a quadratic equation with two solutions for v:

v<sub>1</sub> = (-w<sub>x</sub>/cos(theta<sub>1</sub>) + a<sub>1</sub> + sqrt((w<sub>x</sub>/cos(theta<sub>1</sub>) - a<sub>1</sub>)<sup>2</sup> - (d<sub>1</sub> - w<sub>z</sub> - a<sub>2</sub> + k).(w<sub>z</sub> - d<sub>1</sub> - a<sub>2</sub> + k))) / (d<sub>1</sub> - w<sub>z</sub> - a<sub>2</sub> + k)

Given the configuration space, theta<sub>2</sub> tends to have a positive value, hence we choose v<sub>2</sub>.

With theta<sub>1</sub>, theta<sub>2</sub> and theta<sub>3</sub> known, we can calculate the rotation matrix <sup>0</sup>R<sub>3</sub>, which is the rotation component of the transformation matrix <sup>0</sup>T<sub>3</sub>. Since <sup>0</sup>T<sub>ee</sub> = <sup>0</sup>T<sub>3</sub>.<sup>3</sup>T<sub>ee</sub>, and the DH convention is formulated in such a way that any displacement along an axis is preceded by a rotation around that axis, then the rotation components of the transformation matrices do not depend on the displacements. Hence, we can also state that <sup>0</sup>R<sub>ee</sub> = <sup>0</sup>R<sub>3</sub>.<sup>3</sup>R<sub>ee</sub>. Since <sup>0</sup>R<sub>ee</sub> is given, we can obtain <sup>3</sup>R<sub>ee</sub> = <sup>0</sup>R<sub>3</sub><sup>-1</sup>.<sup>0</sup>R<sub>ee</sub>.

Analytically, we can obtain <sup>3</sup>R<sub>ee</sub>, which is:

NOT | A | ROW
--- | --- | ---
-sin(theta<sub>5</sub>).cos(theta<sub>4</sub>) | sin(theta<sub>4</sub>).cos(theta<sub>6</sub>) + sin(theta<sub>6</sub>).cos(theta<sub>4</sub>).cos(theta<sub>5</sub>) | -sin(theta<sub>4</sub>).sin(theta<sub>6</sub>) + cos(theta<sub>4</sub>).cos(theta<sub>5</sub>).cos(theta<sub>6</sub>)
cos(theta<sub>5</sub>) | sin(theta<sub>5</sub>).sin(theta<sub>6</sub>) | sin(theta<sub>5</sub>).cos(theta<sub>6</sub>)
sin(theta<sub>4</sub>).sin(theta<sub>5</sub>) | -sin(theta<sub>4</sub>).sin(theta<sub>6</sub>).cos(theta<sub>5</sub>) + cos(theta<sub>4</sub>).cos(theta<sub>6</sub>) | -sin(theta<sub>4</sub>).cos(theta<sub>5</sub>).cos(theta<sub>6</sub>) - sin(theta<sub>6</sub>).cos(theta<sub>4</sub>)

Then, the remaining thetas can be calculated as follows:

theta<sub>4</sub> = atan2(<sup>3</sup>R<sub>7<sub>2,0</sub></sub>, -<sup>3</sup>R<sub>7<sub>0,0</sub></sub>)

theta<sub>5</sub> = atan2(sqrt(<sup>3</sup>R<sub>7<sub>2,0</sub></sub><sup>2</sup> + <sup>3</sup>R<sub>7<sub>0,0</sub></sub><sup>2</sup>), <sup>3</sup>R<sub>7<sub>1,0</sub></sub>)

theta<sub>6</sub> = atan2(<sup>3</sup>R<sub>7<sub>1,1</sub></sub>, <sup>3</sup>R<sub>7<sub>1,2</sub></sub>)

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

The code just follows the step previously described. I have tested it and succedded more than 10 times consecutively. Something I might improve if I could spend more time on this project is the rotation around joint_4, which happens when theta<sub>4</sub> is close to _pi_. Although the robot arm do not deviate from the trajectory, these rotations make it reach the destination a little bit slower than it should.

In the code, I commented some parts that I use to derive the analytical expressions for the transformations using sympy (I did some of them by hand, but sympy helped me to check that my calculations were correct. For other calculations, I just relied on the sympy results). Also, I commented some portion of the code where I check that the translation obtained when using the calculated angles theta<sub>2</sub> and theta<sub>3</sub> is correct, and that I choose the appropriate angle versions. Removing the commented portions, the code is actually quite small. The main challenge was the design part, i.e., thinking about and verifying the strategy for calculating the thetas, which is explained in the point 3 of the kinematic analysis of this writeup.

