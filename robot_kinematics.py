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
from math import sin, cos, tan, pi
from numpy.core.numeric import ones
from numpy.lib.function_base import gradient
from scipy import optimize

MAX_ANGLE = math.radians(180)
MIN_ANGLE = math.radians(-180)


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
            int(self.theta1, 10) // 10,
            int(self.theta2, 10) // 10,
            int(self.theta3, 10) // 10,
            int(self.theta4, 10) // 10,
            int(self.gripper, 10) // 10,
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

    @staticmethod
    def npArrayToJointAngles(array):
        return JointAngles(
            array[0],
            array[1],
            array[2],
            array[3],
            0,
        )

    def add(self, other: Type["JointAngles"]):
        return JointAngles(
            self.theta1 + other.theta1,
            self.theta2 + other.theta2,
            self.theta3 + other.theta3,
            self.theta4 + other.theta4,
            self.gripper + other.gripper,
        )

    def toNPArray(self):
        return np.array([self.theta1, self.theta2, self.theta3, self.theta4])

    def constraint_angle(self):
        self.theta1 = constraints(self.theta1)
        self.theta2 = constraints(self.theta2)
        self.theta3 = constraints(self.theta3)
        self.theta4 = constraints(self.theta4)
        self.gripper = constraints(self.gripper)

    # Multiply
    def __mul__(self, other):
        return JointAngles(
            self.theta1 * other,
            self.theta2 * other,
            self.theta3 * other,
            self.theta4 * other,
            self.gripper * other,
        )


@dataclass
class CartesianCoordinates:
    # Cartesian Coordinates
    x: float
    y: float
    z: float

    def return_np(self):
        return np.array([self.x, self.y, self.z])


def inverse_kinematics(
    cartesian_coordinates: CartesianCoordinates, starting_joint_angles: JointAngles
) -> JointAngles:
    """
    Calculates the Joint Angles of the end-effector by using Jacobian
    """
    # TODO: Enforce constraints
    e = cartesian_coordinates
    threshold = 0.1
    current_position = forward_kinematics(starting_joint_angles)
    current_joint_angles = starting_joint_angles
    while (
       distance(current_joint_angles.toNPArray(), e) > threshold
    ):
        J_i = jacobian_inv(current_joint_angles)
        change_cartesian_coordinates = CartesianCoordinates(
            e.x - current_position.x, e.y - current_position.y, e.z - current_position.z
        )
        change_angles = np.matmul(J_i[:, :3], change_cartesian_coordinates.return_np())
        change_angles = JointAngles.npArrayToJointAngles(change_angles * -1)
        # change_angles.constraint_angle()

        current_joint_angles = current_joint_angles.add(change_angles)

        current_position = forward_kinematics(current_joint_angles)
    print(current_joint_angles)

def inv_opt(
    starting_joint_angles: JointAngles, cartesian_coordinates: CartesianCoordinates
):
    """ 
    Minimum distance to target to find theta1, theta2, theta3, theta4
    """
    optimized_joint_angles = optimize.minimize(
        fun=lambda joint_angles: distance(joint_angles, cartesian_coordinates),
        x0=starting_joint_angles.toNPArray(),
        bounds=(
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
        ),
        method="L-BFGS-B",
    )
    optimized_joint_angles = JointAngles.npArrayToJointAngles(optimized_joint_angles.x)
    print(optimized_joint_angles)
    print(forward_kinematics(optimized_joint_angles))
    return optimized_joint_angles


def distance(joint_angles: np.ndarray, target: CartesianCoordinates):
    current: np.ndarray = forward_kinematics(
        JointAngles.npArrayToJointAngles(joint_angles)
    ).return_np()
    target: np.ndarray = target.return_np()
    return np.linalg.norm(current - target)


def jacobian_inv(joint_angles: JointAngles) -> np.array:
    """Calculates the Jacobian Matrix"""
    q1 = joint_angles.theta1
    q2 = joint_angles.theta2
    q3 = joint_angles.theta3
    q4 = joint_angles.theta4
    e = np.array(
        [
            [
                (
                    5.61 * sin(q2)
                    + 6.39 * cos(q2 + q3)
                    + 4.52 * cos(q2 + q3 + q4)
                )
                * sin(q1),
                (
                    6.39 * sin(q2 + q3)
                    + 4.52 * sin(q2 + q3 + q4)
                    - 5.61 * cos(q2)
                )
                * cos(q1),
                (
                    6.39 * sin(q2 + q3)
                    + 4.52 * sin(q2 + q3 + q4)
                )
                * cos(q1),
                4.52 * sin(q2 + q3 + q4) * cos(q1),
            ],
            [
                -(
                    5.61 * sin(q2)
                    + 6.39 * cos(q2 + q3)
                    + 4.52 * cos(q2 + q3 + q4)
                )
                * cos(q1),
                (
                    6.39 * sin(q2 + q3)
                    + 4.52 * sin(q2 + q3 + q4)
                    - 5.61 * cos(q2)
                )
                * sin(q1),
                (
                    6.39 * sin(q2 + q3)
                    + 4.52 * sin(q2 + q3 + q4)
                )
                * sin(q1),
                4.52 * sin(q1) * sin(q2 + q3 + q4),
            ],
            [
                0,
                -5.61 * sin(q2)
                - 6.39 * cos(q2 + q3)
                - 4.52 * cos(q2 + q3 + q4),
                -6.39 * cos(q2 + q3)
                - 4.52 * cos(q2 + q3 + q4),
                -4.52 * cos(q2 + q3 + q4),
            ],
            [0, sin(q1), sin(q1), sin(q1)],
            [0, -cos(q1), -cos(q1), -cos(q1)],
            [1, 0, 0, 0],
        ]
    )
    return np.linalg.pinv(e)


def forward_kinematics(joint_angles: JointAngles) -> CartesianCoordinates:
    """Calculates the Cartesian Coordinates of the end-effector"""
    T = denavit_hartenberg(joint_angles)
    return CartesianCoordinates(T[0, 3], T[1, 3], T[2, 3])


def constraints(
    theta: float,
    theta_min: float = MIN_ANGLE,
    theta_max: float = MAX_ANGLE,
):
    return min(max(theta, theta_min), theta_max)


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
    # joint_angles = joint_angles.degreeToRadian()
    thetas = [
        joint_angles.theta1,
        joint_angles.theta2,
        joint_angles.theta3,
        joint_angles.theta4,
    ]
    d = [413 / 100, 0, 0, 0]
    a = [0, 561 / 100, 639 / 100, 113 / 25]
    alpha = [math.pi / 2, 0, 0, 0]
    offset = [0, math.pi / 2, math.pi / 2, 0]

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


inverse_kinematics( CartesianCoordinates(0, 1, 5), JointAngles(theta1=0, theta2=0, theta3=0, theta4=0, gripper=0),)
inv_opt(JointAngles(20, 6, 2, 23, 4).degreeToRadian(), CartesianCoordinates(4, 4, 5))
# inv_opt(JointAngles(20, 6, 62, 7, 4).degreeToRadian(), CartesianCoordinates(9, 4, 2))
# inv_opt(JointAngles(2, 6, 62, 23, 4).degreeToRadian(), CartesianCoordinates(2, 4, 6))
# inverse_kinematics(cartesian_coordinates=CartesianCoordinates(5, 4, 2), starting_joint_angles=JointAngles(20, 6, 62, 23, 4).degreeToRadian())
