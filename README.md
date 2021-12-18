# Movement for the Lynxmotion 4DOF Robot Arm

## Overview
This repo is for our CMPUT312 Final Project. It contains code for Forward Kinematics and Inverse Kinematics for a 4DOF Robot Arm.

Forward Kinematics is used to calculate the position of the robot arm's end-effector given the joint angles of the arm.

Inverse Kinematics is used to calculate the joint angles given a position in 3D cartesian space.

It also contains code for interfacing with the Lynxmotion 4DOF Robot Arm, as well as to connect to the render engine server.

Main file: `control.py`
    - Contains code for demos: interpolation, and real-time tracking.

## Forward Kinematics

**Denavit-Hartenberg Parameters**



| Joint | <!-- $\color{grey}\theta$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ccolor%7Bgrey%7D%5Ctheta"> | d    | a    | <!-- $\color{grey}\alpha$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ccolor%7Bgrey%7D%5Calpha"> | offset                                                                                                                                                                              |
| ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1     | q1                                                                                                                                                                                  | 4.13 | 0    | <!-- $\color{grey}\pi/2$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ccolor%7Bgrey%7D%5Cpi%2F2">                                                                | 0                                                                                                                                                                                   |
| 2     | q2                                                                                                                                                                                  | 0    | 5.61 | 0                                                                                                                                                                                   | <!-- $\color{grey}\pi/2$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ccolor%7Bgrey%7D%5Cpi%2F2"> |
| 3     | q3                                                                                                                                                                                  | 0    | 6.39 | 0                                                                                                                                                                                   | <!-- $\color{grey}\pi/2$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Ccolor%7Bgrey%7D%5Cpi%2F2"> |
| 4     | q4                                                                                                                                                                                  | 0    | 4.52 | 0                                                                                                                                                                                   | 0                                                                                                                                                                                   |



![Robot workspace](https://github.com/CMPUT312-Final-Project/robot-arm/raw/f5b7601f4e0b0169ba6125f1649eb3fd243f89a8/robot.png)
Robot parameters visualized using the **Robotics Toolbox** [1]

## Inverse Kinematics

**Jacobian Matrix** [2]

<img src="https://render.githubusercontent.com/render/math?math=%5CLarge%20%5Ccolor%7Bgrey%7DJ%3D%0A%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Cleft(5.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20%2B%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20-%205.61%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C-%20%5Cleft(5.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20%2B%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20-%205.61%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Cleft(6.39%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20%2B%204.52%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5Cright)%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%204.52%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%5Csin%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5C%5C0%20%26%20-%205.61%20%5Csin%7B%5Cleft(q_%7B2%7D%20%5Cright)%7D%20-%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%26%20-%206.39%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%5Cright)%7D%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%20%26%20-%204.52%20%5Ccos%7B%5Cleft(q_%7B2%7D%20%2B%20q_%7B3%7D%20%2B%20q_%7B4%7D%20%5Cright)%7D%5C%5C0%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20%5Csin%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C0%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%20%26%20-%20%5Ccos%7B%5Cleft(q_%7B1%7D%20%5Cright)%7D%5C%5C1%20%26%200%20%26%200%20%26%200%5Cend%7Bmatrix%7D%5Cright%5D">



## Acknowledgement

Thanks to Geraldine and Eduardo for providing the [logic to control the LSS servos and the measurements for the arm.](https://github.com/Robotics-Technology/Chess-Robot/blob/master/ArmControl.py)
-   [Geraldine Barreto](http://github.com/geraldinebc)
-   [Eduardo Nunes](https://github.com/EduardoFNA)

## References

[1] P. I. Corke, _Robotics, Vision & Control: Fundamental Algorithms in MATLAB_, Second. Springer, 2017.

[2] A. Addison, “The Ultimate Guide to Jacobian Matrices for Robotics – Automatic Addison.” [https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/](https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/) (accessed Nov. 11, 2021).