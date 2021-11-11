# Movement for the Lynxmotion 4DOF Robot Arm

## Overview
This repo is for our CMPUT312 Final Project. It contains code for Forward Kinematics and Inverse Kinematics for a 4DOF Robot Arm.

Forward Kinematics is used to calculate the position of the robot arm's end-effector given the joint angles of the arm.

Inverse Kinematics is used to calculate the joint angles given a position in 3D cartesian space.

## Forward Kinematics

**Denavit-Hartenberg Parameters**



| Joint |<!-- $\theta$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=%5Ctheta">  | d | a  |<!-- $\alpha$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=%5Calpha"> |offset  |
|--|--|--|--|--|--|
| 1 | q1 | 4.13 | 0 | <!-- $\pi/2$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=%5Cpi%2F2"> | 0 |
| 2 | q2 | 0 | 5.61 | 3 | <!-- $\pi/2$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=%5Cpi%2F2"> |
| 3 | q3 | 0 | 6.39 | 3 | <!-- $\pi/2$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=%5Cpi%2F2"> |
| 4 | q4 | 0 | 4.52 | 3 | 0 |



![Robot workspace](https://github.com/CMPUT312-Final-Project/robot-arm/raw/f5b7601f4e0b0169ba6125f1649eb3fd243f89a8/robot.png)
Robot parameters visualized using the **Robotics Toolbox** [1]

## Inverse Kinematics

**Jacobian Matrix** [2]

<!-- $J=\left[\begin{matrix}\left(5.61 \sin{\left(q_{2} \right)} + 6.39 \cos{\left(q_{2} + q_{3} \right)} + 4.52 \cos{\left(q_{2} + q_{3} + q_{4} \right)}\right) \sin{\left(q_{1} \right)} & \left(6.39 \sin{\left(q_{2} + q_{3} \right)} + 4.52 \sin{\left(q_{2} + q_{3} + q_{4} \right)} - 5.61 \cos{\left(q_{2} \right)}\right) \cos{\left(q_{1} \right)} & \left(6.39 \sin{\left(q_{2} + q_{3} \right)} + 4.52 \sin{\left(q_{2} + q_{3} + q_{4} \right)}\right) \cos{\left(q_{1} \right)} & 4.52 \sin{\left(q_{2} + q_{3} + q_{4} \right)} \cos{\left(q_{1} \right)}\\- \left(5.61 \sin{\left(q_{2} \right)} + 6.39 \cos{\left(q_{2} + q_{3} \right)} + 4.52 \cos{\left(q_{2} + q_{3} + q_{4} \right)}\right) \cos{\left(q_{1} \right)} & \left(6.39 \sin{\left(q_{2} + q_{3} \right)} + 4.52 \sin{\left(q_{2} + q_{3} + q_{4} \right)} - 5.61 \cos{\left(q_{2} \right)}\right) \sin{\left(q_{1} \right)} & \left(6.39 \sin{\left(q_{2} + q_{3} \right)} + 4.52 \sin{\left(q_{2} + q_{3} + q_{4} \right)}\right) \sin{\left(q_{1} \right)} & 4.52 \sin{\left(q_{1} \right)} \sin{\left(q_{2} + q_{3} + q_{4} \right)}\\0 & - 5.61 \sin{\left(q_{2} \right)} - 6.39 \cos{\left(q_{2} + q_{3} \right)} - 4.52 \cos{\left(q_{2} + q_{3} + q_{4} \right)} & - 6.39 \cos{\left(q_{2} + q_{3} \right)} - 4.52 \cos{\left(q_{2} + q_{3} + q_{4} \right)} & - 4.52 \cos{\left(q_{2} + q_{3} + q_{4} \right)}\\0 & \sin{\left(q_{1} \right)} & \sin{\left(q_{1} \right)} & \sin{\left(q_{1} \right)}\\0 & - \cos{\left(q_{1} \right)} & - \cos{\left(q_{1} \right)} & - \cos{\left(q_{1} \right)}\\1 & 0 & 0 & 0\end{matrix}\right]$ --> <img style="transform: translateY(0.1em); background: white !important;" src="https://render.githubusercontent.com/render/math?math=J%3D%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Cleft(5.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20%2B%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20-%205.61%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C-%20%5Cleft(5.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20%2B%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20-%205.61%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%204.52%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5C%5C0%20%26%20-%205.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20-%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%26%20-%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%26%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5C%5C0%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C0%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C1%20%26%200%20%26%200%20%26%200%5Cend%7Bmatrix%7D%5Cright%5D">

## Acknowledgement

Thanks to Geraldine and Eduardo for providing the [logic to control the LSS servos and the measurements for the arm.](https://github.com/Robotics-Technology/Chess-Robot/blob/master/ArmControl.py)
-   [Geraldine Barreto](http://github.com/geraldinebc)
-   [Eduardo Nunes](https://github.com/EduardoFNA)

## References

[1] P. I. Corke, _Robotics, Vision & Control: Fundamental Algorithms in MATLAB_, Second. Springer, 2017.

[2] A. Addison, “The Ultimate Guide to Jacobian Matrices for Robotics – Automatic Addison.” [https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/](https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/) (accessed Nov. 11, 2021).