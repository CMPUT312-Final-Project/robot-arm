###############################################################################
#	Authors:		Geraldine Barreto, Eduardo Nunes
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Description:	Helper code for moving the robot to a given position
#   Url:            https://github.com/Robotics-Technology/Chess-Robot/blob/master/ArmControl.py
###############################################################################

from lss import LSS
from robot_kinematics import JointAngles

def move_to_angle(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, angle: JointAngles):
   
    base.moveCH(angle.theta1, 1000)
    shoulder.moveCH(angle.theta2, 1600)
    elbow.moveCH(angle.theta3, 1600)
    wrist.moveCH(angle.theta4, 1000)
    gripper.moveCH(0, 500)
    arrived = False
    issue = 0
    i = 0

    # Check if they reached the requested position
    while arrived == False and issue == 0:
        bStat = base.getStatus()
        sStat = shoulder.getStatus()
        eStat = elbow.getStatus()
        wStat = wrist.getStatus()
        gStat = gripper.getStatus()
        # If a status is None print message if it continues to be None return issue 1
        if (bStat is None or sStat is None or eStat is None or wStat is None or gStat is None):
            print("- Unknown status")
            i = i + 1
            if (i >= 5):
                print("- Issue detected")
                issue = 1
        # If the statuses aren't None check their values
        else:
            # If a servo is Outside limits, Stuck, Blocked or in Safe Mode before it reaches the requested position reset the servos and return issue 1
            if (int(bStat)>6 or int(sStat)>6 or int(eStat)>6 or int(wStat)>6 or int(gStat)>6):
                print("- Issue detected")
                issue = 1
            # If all the servos are holding positions check if they have arrived
            elif (int(bStat)==6 and int(sStat)==6 and int(eStat)==6 and int(wStat)==6 and int(gStat)==6):
                bPos = base.getPosition()
                sPos = shoulder.getPosition()
                ePos = elbow.getPosition()
                wPos = wrist.getPosition()
                # If any position is None
                if (bPos is None or sPos is None or ePos is None or wPos is None):
                    print("- Unknown position")
                # If they are holding in a different position than the requested one return issue 2
                elif (abs(int(bPos)-angle.theta1)>20 or abs(int(sPos)-angle.theta2)>50 or abs(int(ePos)-angle.theta3)>50 or abs(int(wPos)-angle.theta4)>20):
                    sStat = shoulder.getStatus()
                    eStat = elbow.getStatus()
                    # Re-check shoulder and elbow status and positions
                    if (int(sStat)==6 and int(eStat)==6):
                        sPos = shoulder.getPosition()
                        ePos = elbow.getPosition()
                        if (sPos is None or ePos is None):
                            print("- Unknown position")
                        elif (abs(int(sPos)-angle.theta2)>40 or abs(int(ePos)-angle.theta3)>40):
                                print("- Obstacle detected")
                                issue = 2
                else:
                    print("- Arrived\n")
                    arrived = True

    return(arrived, issue)
