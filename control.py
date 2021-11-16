###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	This is a library for controlling the robot arm.
###############################################################################

import asyncio
from math import pi
import time
from info_client import sendCartesian
from misc import Theta
from robot_kinematics import CartesianCoordinates, inverse_kinematics
import lss
import lss_const as lssc
from robot_kinematics import JointAngles, forward_kinematics
import sys, signal
from lss import LSS
from util import motor_angles_to_joint_angles
from LSS_movement import move_to_angle
from threading import Thread

def _handler(signal, frame):
    print("\nYou pressed Ctrl+C!")
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
allMotors.setMaxSpeed(30)
base.setMaxSpeed(20)
# shoulder.setMotionControlEnabled(0)
# elbow.setMotionControlEnabled(0)
event_loop_a = asyncio.new_event_loop()

def track_arm_position(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS):

    # Start thread to read sensor data
    Thread(target=sensor, args=(base, shoulder, elbow, wrist, gripper), daemon=True).start()

    while True:
        input_key = input("Select mode (h,l):")
        if input_key == "h":
            base.hold()
            shoulder.hold()
            elbow.hold()
            wrist.hold()
            gripper.hold()
            allMotors.hold()
        else:
            base.limp()
            shoulder.limp()
            elbow.limp()
            wrist.limp()
            gripper.limp()
            allMotors.limp()
        

def sensor(base, shoulder, elbow, wrist, gripper):
    asyncio.set_event_loop(event_loop_a)
    pos = JointAngles(0, 0, 0, 0, 0).degree_to_radian()

    while True:
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
            # if pos.has_changed(lastPos, 0.001):
            coordinates: CartesianCoordinates = forward_kinematics(pos)
            asyncio.get_event_loop().run_until_complete(sendCartesian(coordinates))

def move_arm(
    target: CartesianCoordinates,
    base: LSS,
    shoulder: LSS,
    elbow: LSS,
    wrist: LSS,
    gripper: LSS,
    allMotors: LSS,
    divisions=1,
):
    current_theta: JointAngles = motor_angles_to_joint_angles(
        base, shoulder, elbow, wrist, gripper
    )
    current_pos: CartesianCoordinates = forward_kinematics(current_theta)

    delta_pos = (target - current_pos) / divisions

    for i in range(divisions):
        new_pos = current_pos + delta_pos
        move_angles: JointAngles = inverse_kinematics(new_pos, current_theta)
        move_angles = move_angles.radian_to_degree().to_motor_degrees()

        a = move_to_angle(base, shoulder, elbow, wrist, gripper, move_angles)
        current_theta = motor_angles_to_joint_angles(
            base, shoulder, elbow, wrist, gripper
        )
        current_pos = forward_kinematics(current_theta)


def control_with_keys(
    base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS
):

    while True:
        initial_pos = forward_kinematics(
            motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
        )
        input_key = input("Select")
        if input_key == "w":
            move_arm(
                initial_pos + CartesianCoordinates(0, 0, 2),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "s":
            move_arm(
                initial_pos - CartesianCoordinates(0, 0, 2),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "a":
            move_arm(
                initial_pos + CartesianCoordinates(2, 0, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "d":
            move_arm(
                initial_pos - CartesianCoordinates(2, 0, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "r":
             move_arm(
                initial_pos + CartesianCoordinates(0, 2, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "l":
            move_arm(
                initial_pos - CartesianCoordinates(0, 2, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "q":
            break


# move_arm(CartesianCoordinates(x=3.505244735627988, y=-7.872908577754623, z=2.5552216211064342), base, shoulder, elbow, wrist, gripper, allMotors)
track_arm_position(base, shoulder, elbow, wrist, gripper, allMotors)

# control_with_keys(base, shoulder, elbow, wrist, gripper, allMotors)
