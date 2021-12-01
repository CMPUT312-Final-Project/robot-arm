###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	Forward and Inverse Kinematics for 4-DoF Arm
#   Bibligraphy:
#   [1] S. R. Buss, “Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods,” p. 19.
###############################################################################

from dataclasses import dataclass
import time
from typing import List, Type
import numpy as np
import math
from math import sin, cos, pi
from numpy.core.function_base import linspace
from scipy import optimize, interpolate
import matplotlib.pyplot as plt

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
    target: CartesianCoordinates, starting_joint_angles: JointAngles,
    attempts: int = 1000,
) -> JointAngles:
    """
    Calculates the Joint Angles of the end-effector by using Jacobian and enforcing the constraints
    """
    e = target
    invalid = False
    threshold = 0.1
    current_position = forward_kinematics(starting_joint_angles)
    current_joint_angles = starting_joint_angles
    i = 0
    while distance(current_joint_angles.to_np_array(), e) > threshold and i < attempts:
        J = jacobian(current_joint_angles)
        J_t = J.transpose()  # [1] Using transpose method with small alpha
        change_cartesian_coordinates = e - current_position
        change_angles = np.matmul(J_t[:, :3], change_cartesian_coordinates.return_np())
        alpha = 0
        # Alpha should be close to change_cartesian_coordinates
        change = change_cartesian_coordinates.return_np()
        # Inner product of change and change
        J_J_t_change = J[:3,:] @ change_angles 
        numerator = np.inner(change, J_J_t_change)
        denominator = np.inner(J_J_t_change, J_J_t_change)
        alpha = (numerator/denominator) # [1] Formula for alpha
        change_angles = JointAngles.np_array_to_joint_angles(change_angles * alpha)
        prev_joint_angles = current_joint_angles
        current_joint_angles = current_joint_angles + change_angles
        i += 1
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
                (
                    10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * cos(q4 - 0.1072 * pi)
                    + 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * sin(q3 + 0.4414 * pi)
                    - 10.59
                    * sin(q2 + q3)
                    * sin(q4 - 0.1072 * pi)
                    - 16.3005
                    * cos(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    - 14.24
                    * cos(q2 + 0.5585 * pi)
                )
                * sin(q1),
                -(
                    -10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    + 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    + 14.24
                    * sin(q2 + 0.5585 * pi)
                    - 10.59
                    * sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                    + 16.3005
                    * sin(q3 + 0.4414 * pi)
                    * cos(q2 + 0.5585 * pi)
                )
                * cos(q1),
                (
                    10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    - 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    + 10.59
                    * sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                    - 16.3005
                    * sin(q3 + 0.4414 * pi)
                    * cos(q2 + 0.5585 * pi)
                )
                * cos(q1),
                10.59
                * (
                    (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    + sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                )
                * cos(q1),
            ],
            [
                (
                    -10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * cos(q4 - 0.1072 * pi)
                    - 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * sin(q3 + 0.4414 * pi)
                    + 10.59
                    * sin(q2 + q3)
                    * sin(q4 - 0.1072 * pi)
                    + 16.3005
                    * cos(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    + 14.24
                    * cos(q2 + 0.5585 * pi)
                )
                * cos(q1),
                -(
                    -10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    + 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    + 14.24
                    * sin(q2 + 0.5585 * pi)
                    - 10.59
                    * sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                    + 16.3005
                    * sin(q3 + 0.4414 * pi)
                    * cos(q2 + 0.5585 * pi)
                )
                * sin(q1),
                (
                    10.59
                    * (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    - 16.3005
                    * sin(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                    + 10.59
                    * sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                    - 16.3005
                    * sin(q3 + 0.4414 * pi)
                    * cos(q2 + 0.5585 * pi)
                )
                * sin(q1),
                10.59
                * (
                    (
                        sin(q2 + 0.5585 * pi)
                        * sin(q3 + 0.4414 * pi)
                        - cos(q2 + 0.5585 * pi)
                        * cos(q3 + 0.4414 * pi)
                    )
                    * sin(q4 - 0.1072 * pi)
                    + sin(q2 + q3)
                    * cos(q4 - 0.1072 * pi)
                )
                * sin(q1),
            ],
            [
                0,
                -10.59
                * (
                    sin(q2 + 0.5585 * pi)
                    * sin(q3 + 0.4414 * pi)
                    - cos(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                )
                * cos(q4 - 0.1072 * pi)
                - 16.3005
                * sin(q2 + 0.5585 * pi)
                * sin(q3 + 0.4414 * pi)
                + 10.59
                * sin(q2 + q3)
                * sin(q4 - 0.1072 * pi)
                + 16.3005
                * cos(q2 + 0.5585 * pi)
                * cos(q3 + 0.4414 * pi)
                + 14.24
                * cos(q2 + 0.5585 * pi),
                -10.59
                * (
                    sin(q2 + 0.5585 * pi)
                    * sin(q3 + 0.4414 * pi)
                    - cos(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                )
                * cos(q4 - 0.1072 * pi)
                - 16.3005
                * sin(q2 + 0.5585 * pi)
                * sin(q3 + 0.4414 * pi)
                + 10.59
                * sin(q2 + q3)
                * sin(q4 - 0.1072 * pi)
                + 16.3005
                * cos(q2 + 0.5585 * pi)
                * cos(q3 + 0.4414 * pi),
                -10.59
                * (
                    sin(q2 + 0.5585 * pi)
                    * sin(q3 + 0.4414 * pi)
                    - cos(q2 + 0.5585 * pi)
                    * cos(q3 + 0.4414 * pi)
                )
                * cos(q4 - 0.1072 * pi)
                + 10.59
                * sin(q2 + q3)
                * sin(q4 - 0.1072 * pi),
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
    +---+-----------+-----------+-----------+-----------+--------------------+
    | j |     theta |         d |         a |     alpha |    offset          |
    +---+-----------+-----------+-----------+-----------+--------------------+
    |  1|         q1|    10.4902|          0|       pi/2|          0         |
    |  2|         q2|          0|    14.2400|          0| pi/2 + 10.54*pi/180|
    |  3|         q3|          0|    16.3006|          0| pi/2 - 10.54*pi/180|
    |  4|         q4|          0|    10.5900|          0| -19.3*pi/180       |
    +---+-----------+-----------+-----------+-----------+--------------------+
    """
    # joint_angles = joint_angles.degree_to_radian()
    thetas = [
        joint_angles.theta1,
        joint_angles.theta2,
        joint_angles.theta3,
        joint_angles.theta4,
    ]
    d = [10.4902, 0, 0, 0]
    a = [0, 14.24, 16.3006, 10.5900]
    alpha = [math.pi / 2, 0, 0, 0]
    offset = [
        0,
        (math.pi / 2) + 10.54 * math.pi / 180,
        math.pi / 2 - 10.54 * math.pi / 180,
        -19.3 * math.pi / 180,
    ]

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

def joint_interpolation(starting_angle: JointAngles, end_angle: JointAngles, segments=200)->List[JointAngles]:
    linspace_theta1 = np.linspace(starting_angle.theta1, end_angle.theta1, segments)
    linspace_theta2 = np.linspace(starting_angle.theta2, end_angle.theta2, segments)
    linspace_theta3 = np.linspace(starting_angle.theta3, end_angle.theta3, segments)
    linspace_theta4 = np.linspace(starting_angle.theta4, end_angle.theta4, segments)
    interpolated_angles = []
    
    for i in range(len(linspace_theta1)):
        interpolated_angles.append(JointAngles(linspace_theta1[i], linspace_theta2[i], linspace_theta3[i], linspace_theta4[i],0))
    
    return interpolated_angles

def cartesian_interploation(starting_point: CartesianCoordinates, end_point: CartesianCoordinates, segments=200)->List[CartesianCoordinates]:
    linspace_x = np.linspace(starting_point.x, end_point.x, segments)
    linspace_y = np.linspace(starting_point.y, end_point.y, segments)
    linspace_z = np.linspace(starting_point.z, end_point.z, segments)
    interpolated_points = []
    
    for i in range(len(linspace_x)):
        interpolated_points.append(CartesianCoordinates(linspace_x[i], linspace_y[i], linspace_z[i]))
    
    return interpolated_points
