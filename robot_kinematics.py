###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	Forward and Inverse Kinematics for 4-DoF Arm
###############################################################################

from dataclasses import dataclass
from typing import Type
import numpy as np
import math


@dataclass
class JointAngles:
    # Joint Angles and Gripper Aperture
    theta1: float
    theta2: float
    theta3: float
    theta4: float
    gripper: float

    # Return false if any joint angle is None
    def isValid(self):
        return (
            self.theta1 is not None
            and self.theta2 is not None
            and self.theta3 is not None
            and self.theta4 is not None
            and self.gripper is not None
        )

    def intify(self):
        return JointAngles(
            int(self.theta1, 10)//10,
            int(self.theta2, 10)//10,
            int(self.theta3, 10)//10,
            int(self.theta4, 10)//10,
            int(self.gripper, 10)//10,
        )

    def hasChanged(self, other: Type["JointAngles"]):
        # Treshold for change == 2 degrees
        return (
            abs(self.theta1 - other.theta1) > 2
            or abs(self.theta2 - other.theta2) > 2
            or abs(self.theta3 - other.theta3) > 2
            or abs(self.theta4 - other.theta4) > 2
            or abs(self.gripper - other.gripper) > 2
        )
    
    def degreeToRadian(self):
        return JointAngles(
            math.radians(self.theta1),
            math.radians(self.theta2),
            math.radians(self.theta3),
            math.radians(self.theta4),
            math.radians(self.gripper),
        )


@dataclass
class CartesianCoordinates:
    # Cartesian Coordinates
    x: float
    y: float
    z: float


def forward_kinematics(joint_angles: JointAngles) -> CartesianCoordinates:
    """Calculates the Cartesian Coordinates of the end-effector"""
    T = denavit_hartenberg(joint_angles)
    return CartesianCoordinates(T[0, 3], T[1, 3], T[2, 3])


def denavit_hartenberg(joint_angles: JointAngles):
    """
    The following function calculates the Denavit-Hartenberg Transformation Matrix for a 4-DoF Arm
    with the following parameters:
    +---+-----------+-----------+-----------+-----------+-----------+
    | j |     theta |         d |         a |     alpha |    offset |
    +---+-----------+-----------+-----------+-----------+-----------+
    |  1|         q1|    413/100|          0|       pi/2|          0|
    |  2|         q2|          0|    561/100|          0|       pi/2|
    |  3|         q3|          0|    639/100|          0|       pi/2|
    |  4|         q4|          0|     113/25|          0|          0|
    +---+-----------+-----------+-----------+-----------+-----------+
    """
    joint_angles = joint_angles.degreeToRadian()
    thetas = [
        joint_angles.theta1,
        joint_angles.theta2,
        joint_angles.theta3,
        joint_angles.theta4,
    ]
    d = [413 / 100, 0, 0, 0]
    a = [0, 561 / 100, 639 / 100, 113 / 25]
    alpha = [math.pi / 2, 0, 0, 0]
    offset = [0, math.pi/2, math.pi/2, 0]

    T = np.identity(4)
    for i in range(len(thetas)):
        T = np.matmul(
            T, transformation_matrix(thetas[i], d[i], a[i], alpha[i], offset[i])
        )
    return T


def transformation_matrix(
    theta: float, d: float, a: float, alpha: float, offset: float
) -> np.array:
    """Calculates the Transformation Matrix"""
    theta = theta + offset
    return np.array(
        [
            [
                math.cos(theta),
                -math.sin(theta) * math.cos(alpha),
                math.sin(theta) * math.sin(alpha),
                a * math.cos(theta),
            ],
            [
                math.sin(theta),
                math.cos(theta) * math.cos(alpha),
                -math.cos(theta) * math.sin(alpha),
                a * math.sin(theta),
            ],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )
