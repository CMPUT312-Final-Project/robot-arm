###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	This is a library for controlling the robot arm.
###############################################################################

from math import pi
import time
from robot_kinematics import CartesianCoordinates, inverse_kinematics
import lss
import lss_const as lssc
from robot_kinematics import JointAngles, forward_kinematics
import sys, signal
from lss import LSS
from LSS_movement import move_to_angle

def _handler(signal, frame):
    print('\nYou pressed Ctrl+C!')
    lss.closeBus()
    sys.exit(0)

signal.signal(signal.SIGINT, _handler)

PORT = "/dev/tty.usbserial-A10KM8FE"
lss.initBus(PORT, lssc.LSS_DefaultBaud)
base = LSS(1)
shoulder = LSS(2)
elbow = LSS(3)
wrist = LSS(4)
gripper = LSS(5)
allMotors = LSS(254)

# Settings from LSS_movement.py
allMotors.setAngularHoldingStiffness(0)
allMotors.setMaxSpeed(100)
base.setMaxSpeed(60)
shoulder.setMotionControlEnabled(0)
elbow.setMotionControlEnabled(0)

def track_arm_position(base, shoulder, elbow, wrist, gripper, allMotors):
    base.move(0)
    shoulder.move(0)
    elbow.move(0)
    wrist.move(0)
    gripper.move(0)
    allMotors.move(0)

    base.limp()
    shoulder.limp()
    elbow.limp()
    wrist.limp()
    gripper.limp()
    allMotors.limp()

    ye = JointAngles(
        base.getPosition(),
        shoulder.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        gripper.getPosition(),
    )
    print(ye.from_motor_degrees())
    pos = JointAngles(0, 0, 0, 0, 0).degree_to_radian()

    while True:
        time.sleep(0.02)
        lastPos = pos
        posRead = JointAngles(
        base.getPosition(),
        shoulder.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        gripper.getPosition(),
    )
        if posRead.is_valid():
            pos = posRead.from_motor_degrees()
            pos = pos.degree_to_radian()
            if pos.has_changed(lastPos, 0.001):
                coordinates: CartesianCoordinates = forward_kinematics(pos)
                print(pos)
                print(coordinates)

def move_arm(target: CartesianCoordinates, base: LSS, shoulder:LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS, divisions=3):
    current_theta: JointAngles = motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
    current_pos: CartesianCoordinates = forward_kinematics(current_theta)
    
    delta_pos = (target - current_pos)/divisions
    
    for i in range(divisions):
        new_pos = current_pos + delta_pos
        move_angles: JointAngles = inverse_kinematics(new_pos, current_theta) 
        move_angles = move_angles.radian_to_degree().to_motor_degrees()
        
        a = move_to_angle(base, shoulder, elbow, wrist, gripper, move_angles)
        print(a)
        current_theta = motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
        current_pos = forward_kinematics(current_theta)
    

def motor_angles_to_joint_angles(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS):
    return JointAngles(
        base.getPosition(),
        shoulder.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        gripper.getPosition(),
    ).from_motor_degrees().degree_to_radian()
    
move_arm(CartesianCoordinates(x=3.505244735627988, y=-7.872908577754623, z=2.5552216211064342), base, shoulder, elbow, wrist, gripper, allMotors)
#track_arm_position(base, shoulder, elbow, wrist, gripper, allMotors)            
