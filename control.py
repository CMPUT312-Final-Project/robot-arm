###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	This is a library for controlling the robot arm.
###############################################################################

import time
from robot_kinematics import CartesianCoordinates
import lss
import lss_const as lssc
from robot_kinematics import JointAngles, forward_kinematics

PORT = "/dev/tty.usbserial-A10KM8FE"
lss.initBus(PORT, lssc.LSS_DefaultBaud)
base = lss.LSS(1)
shoulder = lss.LSS(2)
elbow = lss.LSS(3)
wrist = lss.LSS(4)
gripper = lss.LSS(5)
allMotors = lss.LSS(254)

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

pos = JointAngles(0, 0, 0, 0, 0)
while 1:
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
        pos = posRead.intify()

        if pos.hasChanged(lastPos):
            coordinates: CartesianCoordinates = forward_kinematics(pos)
            print(coordinates)
