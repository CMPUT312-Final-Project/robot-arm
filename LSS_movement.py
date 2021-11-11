###############################################################################
#	Authors:		Geraldine Barreto, Eduardo Nunes
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Description:	Helper code for moving the robot with given angles
#   Url:            https://github.com/Robotics-Technology/Chess-Robot/blob/master/ArmControl.py
###############################################################################

import time
from lss import LSS
from robot_kinematics import JointAngles

def move_to_angle(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, angle: JointAngles):
    
    start = time.time()
    print("Moving to angle: ", angle)
    base.moveCH(angle.theta1, 1000)
    shoulder.moveCH(angle.theta2, 1600)
    elbow.moveCH(angle.theta3, 1600)
    wrist.moveCH(angle.theta4, 1000)
    gripper.moveCH(angle.gripper, 500)
    print("Done moving to angle: ", angle)
    print("Time: ", time.time() - start)
    arrived = False
    # Check if they reached the requested position
    while arrived == False:
        print("Checking if arrived")
        start = time.time()
        base_stat = base.getStatus()
        shoulder_stat = shoulder.getStatus()
        elbow_stat = elbow.getStatus()
        wrist_stat = wrist.getStatus()
        print("Time to get status: ", time.time() - start, base_stat)
        stats = [base_stat, shoulder_stat, elbow_stat, wrist_stat]
        if stats.count('6') == len(stats):
            arrived = True

    return(arrived)
