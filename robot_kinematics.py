###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	Forward and Inverse Kinematics for 4-DoF Arm
###############################################################################

from dataclasses import dataclass
import time
from typing import Type
import numpy as np
import math
from math import sin, cos
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

    def __init__(self, theta1, theta2, theta3, theta4, gripper):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.gripper = gripper
        if not self.is_valid():
            raise ValueError("Invalid JointAngles")

    # Return false if any joint angle is None
    def is_valid(self):
        return (
            self.theta1 is not None
            and self.theta2 is not None
            and self.theta3 is not None
            and self.theta4 is not None
            and self.gripper is not None
        )

    def from_motor_degrees(self):
        return JointAngles(
            int(self.theta1, 10) // 10,
            int(self.theta2, 10) // 10,
            int(self.theta3, 10) // 10,
            int(self.theta4, 10) // 10,
            int(self.gripper, 10) // 10,
        )

    def to_motor_degrees(self):
        return JointAngles(
            int(self.theta1 * 10),
            int(self.theta2 * 10),
            int(self.theta3 * 10),
            int(self.theta4 * 10),
            int(self.gripper * 10),
        )

    def has_changed(self, other: Type["JointAngles"], proximity: float = 2):
        # Treshold for change == 2 degrees
        return (
            abs(self.theta1 - other.theta1) > proximity
            or abs(self.theta2 - other.theta2) > proximity
            or abs(self.theta3 - other.theta3) > proximity
            or abs(self.theta4 - other.theta4) > proximity
            # or abs(self.gripper - other.gripper) > 2
        )

    def degree_to_radian(self):
        return JointAngles(
            math.radians(self.theta1),
            math.radians(self.theta2),
            math.radians(self.theta3),
            math.radians(self.theta4),
            math.radians(self.gripper),
        )

    def radian_to_degree(self):
        return JointAngles(
            math.degrees(self.theta1),
            math.degrees(self.theta2),
            math.degrees(self.theta3),
            math.degrees(self.theta4),
            math.degrees(self.gripper),
        )

    @staticmethod
    def np_array_to_joint_angles(array):
        return JointAngles(
            array[0],
            array[1],
            array[2],
            array[3],
            0,
        )

    def to_np_array(self):
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

    def __add__(self, other: Type["JointAngles"]):
        return JointAngles(
            self.theta1 + other.theta1,
            self.theta2 + other.theta2,
            self.theta3 + other.theta3,
            self.theta4 + other.theta4,
            self.gripper + other.gripper,
        )

    def __mod__(self, other):
        return JointAngles(
            self.theta1 % other,
            self.theta2 % other,
            self.theta3 % other,
            self.theta4 % other,
            self.gripper % other,
        )


@dataclass
class CartesianCoordinates:
    # Cartesian Coordinates
    x: float
    y: float
    z: float

    def return_np(self):
        return np.array([self.x, self.y, self.z])

    def __sub__(self, other):
        return CartesianCoordinates(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z,
        )

    def __add__(self, other: Type["CartesianCoordinates"]):
        return CartesianCoordinates(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z,
        )

    # Divide by int
    def __truediv__(self, other: int):
        return CartesianCoordinates(
            self.x / other,
            self.y / other,
            self.z / other,
        )


def inverse_kinematics(
    cartesian_coordinates: CartesianCoordinates, starting_joint_angles: JointAngles
) -> JointAngles:
    """
    Calculates the Joint Angles of the end-effector by using Jacobian
    """
    # TODO: Enforce constraints
    e = cartesian_coordinates
    invalid = False
    threshold = 0.1
    current_position = forward_kinematics(starting_joint_angles)
    current_joint_angles = starting_joint_angles
    while distance(current_joint_angles.to_np_array(), e) > threshold:
        J_t = jacobian(current_joint_angles).transpose()
        change_cartesian_coordinates = e - current_position
        change_angles = np.matmul(J_t[:, :3], change_cartesian_coordinates.return_np())
        change_angles = JointAngles.np_array_to_joint_angles(change_angles * 0.005)
        prev_joint_angles = current_joint_angles
        current_joint_angles = current_joint_angles + change_angles

        current_position = forward_kinematics(current_joint_angles)
        current_joint_angles.constraint_angle()
        if current_joint_angles.has_changed(prev_joint_angles, proximity=0.0001):
            current_position = forward_kinematics(current_joint_angles)
        else:
            print("Solution out of range")
            invalid = True
            break
    if invalid:
        raise ValueError("Solution out of range")
    return current_joint_angles


def inv_opt(
    cartesian_coordinates: CartesianCoordinates, starting_joint_angles: JointAngles
):
    """
    Minimum distance to target to find theta1, theta2, theta3, theta4
    """
    optimized_joint_angles = optimize.minimize(
        fun=lambda joint_angles: distance(joint_angles, cartesian_coordinates),
        x0=starting_joint_angles.to_np_array(),
        bounds=(
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
            (MIN_ANGLE, MAX_ANGLE),
        ),
        method="L-BFGS-B",
    )
    optimized_joint_angles = JointAngles.np_array_to_joint_angles(
        optimized_joint_angles.x
    )
    return optimized_joint_angles


def distance(joint_angles: np.ndarray, target: CartesianCoordinates):
    current: np.ndarray = forward_kinematics(
        JointAngles.np_array_to_joint_angles(joint_angles)
    ).return_np()
    target: np.ndarray = target.return_np()
    return np.linalg.norm(current - target)


def jacobian_inv(joint_angles: JointAngles) -> np.array:
    """Calculates the Jacobian Inverse Matrix"""
    e = jacobian(joint_angles)
    return np.linalg.pinv(e)


def jacobian(joint_angles: JointAngles) -> np.array:
    """Calculates the Jacobian Matrix"""
    q1 = joint_angles.theta1
    q2 = joint_angles.theta2
    q3 = joint_angles.theta3
    q4 = joint_angles.theta4
    e = np.array(
        [
            [
                (5.61 * sin(q2) + 6.39 * cos(q2 + q3) + 4.52 * cos(q2 + q3 + q4))
                * sin(q1),
                (6.39 * sin(q2 + q3) + 4.52 * sin(q2 + q3 + q4) - 5.61 * cos(q2))
                * cos(q1),
                (6.39 * sin(q2 + q3) + 4.52 * sin(q2 + q3 + q4)) * cos(q1),
                4.52 * sin(q2 + q3 + q4) * cos(q1),
            ],
            [
                -(5.61 * sin(q2) + 6.39 * cos(q2 + q3) + 4.52 * cos(q2 + q3 + q4))
                * cos(q1),
                (6.39 * sin(q2 + q3) + 4.52 * sin(q2 + q3 + q4) - 5.61 * cos(q2))
                * sin(q1),
                (6.39 * sin(q2 + q3) + 4.52 * sin(q2 + q3 + q4)) * sin(q1),
                4.52 * sin(q1) * sin(q2 + q3 + q4),
            ],
            [
                0,
                -5.61 * sin(q2) - 6.39 * cos(q2 + q3) - 4.52 * cos(q2 + q3 + q4),
                -6.39 * cos(q2 + q3) - 4.52 * cos(q2 + q3 + q4),
                -4.52 * cos(q2 + q3 + q4),
            ],
            [0, sin(q1), sin(q1), sin(q1)],
            [0, -cos(q1), -cos(q1), -cos(q1)],
            [1, 0, 0, 0],
        ]
    )
    return e


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
    # joint_angles = joint_angles.degree_to_radian()
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


def test_implimentation():
    # time it 
    
    target_j = JointAngles(
        theta1=0.9250245035569946,
        theta2=-0.10471975511965978,
        theta3=0.4014257279586958,
        theta4=1.1868238913561442,
        gripper=-0.017453292519943295,
    )
    starting_j = JointAngles(0, 0, 0, 0, 0)
    x = CartesianCoordinates(
        x=-0.5617379012604284, y=-12.726585837836798, z=3.3382126044630382
    )
    start = time.time()
    for i in range(1000):
        j = inverse_kinematics(x, starting_j)
    end = time.time()
    print(end - start)
    
    start = time.time()
    for i in range(1000):
        j = inv_opt(x, starting_j)
    end = time.time()
    print(end - start)
    

# test_implimentation()