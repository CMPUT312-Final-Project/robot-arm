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

def _handler(signal, frame):
    print('\nYou pressed Ctrl+C!')
    lss.closeBus()
    sys.exit(0)

signal.signal(signal.SIGINT, _handler)

PORT = "/dev/tty.usbserial-A10KM8FE"
lss.initBus(PORT, lssc.LSS_DefaultBaud)
base = lss.LSS(1)
shoulder = lss.LSS(2)
elbow = lss.LSS(3)
wrist = lss.LSS(4)
gripper = lss.LSS(5)
allMotors = lss.LSS(254)

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
    print(ye.fromMotorDegrees())
    pos = JointAngles(0, 0, 0, 0, 0).degreeToRadian()

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
        if posRead.isValid():
            pos = posRead.fromMotorDegrees()
            pos = pos.degreeToRadian()
            if pos.hasChanged(lastPos, 0.001):
                coordinates: CartesianCoordinates = forward_kinematics(pos)
                print(pos)
                print(coordinates)

def move_arm(target: CartesianCoordinates, base: lss.LSS, shoulder:lss.LSS, elbow: lss.LSS, wrist: lss.LSS, gripper: lss.LSS, allMotors: lss.LSS, divisions=20):
    current_theta: JointAngles = motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
    current_pos: CartesianCoordinates = forward_kinematics(current_theta)
    
    delta_pos = (target - current_pos)/divisions
    
    for i in range(divisions):
        new_pos = current_pos + delta_pos
        move_angles: JointAngles = inverse_kinematics(new_pos, current_theta) 
        move_angles = move_angles.radianToDegree().toMotorDegrees()
        
        base.move(move_angles.theta1)
        shoulder.move(move_angles.theta2)
        elbow.move(move_angles.theta3)
        wrist.move(move_angles.theta4)
        
        current_theta = motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
        

def motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper):
    return JointAngles(
        base.getPosition(),
        shoulder.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        gripper.getPosition(),
    ).fromMotorDegrees().degreeToRadian()
    
move_arm(CartesianCoordinates(x=-3.612222531626318, y=-4.7935757603360045, z=3.288225926371776), base, shoulder, elbow, wrist, gripper, allMotors)
            
