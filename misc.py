###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	Derive the Jacobian Matrix from the Denavit-Hartenberg Transformation Matrices
###############################################################################

from sympy import *

t = Symbol("t", real=True)

init_printing(use_unicode=True)


class Theta(Function):
    """
    Theta Function
    """

    nargs = 2
    is_real = True
    is_Function = True
    is_commutative = True


thetas = [
    symbols("q1", real=True),  # Theta(symbols("q1"), t),
    symbols("q2", real=True),  # Theta(symbols("q2"), t),
    symbols("q3", real=True),  # Theta(symbols("q3"), t),
    symbols("q4", real=True),  # Theta(symbols("q4"), t),
]


def jacobian_matrix() -> Matrix:
    """Calculates the Jacobian Matrix"""
    T4 = denavit_hartenberg(4)
    d_4 = T4[:3, 3]

    J: Matrix = ones(6, 4)
    for i in range(4):
        if i == 0:
            # pretty_print(Matrix([[0], [0], [1]]).cross(d_4))

            J[:3, i] = Matrix([[0], [0], [1]]).cross(d_4)
            J[3:, i] = Matrix([[0], [0], [1]])
        else:
            T_i: Matrix = denavit_hartenberg(i)
            d_i = T_i[:3, 3]
            R_i = T_i.copy()
            R_i = R_i[:3, :3]
            J[:3, i] = (R_i @ Matrix([[0], [0], [1]])).cross(d_4 - d_i)
            J[3:, i] = R_i @ Matrix([[0], [0], [1]])
    J.simplify()
    pretty_print(J)
    print_latex(J)


def denavit_hartenberg(index: int):
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

    d = [
        413 / 100,
        0,
        0,
        0,
    ]
    a = [
        0,
        561 / 100,
        639 / 100,
        113 / 25,
    ]
    offset = [0, pi / 2, pi / 2, 0]
    alpha = [pi / 2, 0, 0, 0]

    T = eye(4)
    for i in range(index):
        T = T * transformation_matrix(thetas[i], d[i], a[i], alpha[i], offset[i])
    return T


def transformation_matrix(
    theta: Symbol, d: Symbol, a: float, alpha: float, offset: Symbol
) -> Matrix:
    """Calculates the Transformation Matrix"""
    theta = theta + offset
    return Matrix(
        [
            [
                cos(theta),
                -sin(theta) * cos(alpha),
                sin(theta) * sin(alpha),
                a * cos(theta),
            ],
            [
                sin(theta),
                cos(theta) * cos(alpha),
                -cos(theta) * sin(alpha),
                a * sin(theta),
            ],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


def translation_matrix(x: float, y: float, z: float) -> Matrix:
    """Calculates the Translation Matrix"""
    return Matrix(
        [
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ]
    )


def rotation_matrix(axis: str, angle: float) -> Matrix:
    """Calculates the Rotation Matrix"""
    if axis == "x":
        return Matrix(
            [
                [
                    1,
                    0,
                    0,
                ],
                [0, cos(angle), -sin(angle)],
                [0, sin(angle), cos(angle)],
            ]
        )
    elif axis == "y":
        return Matrix(
            [
                [
                    cos(angle),
                    0,
                    sin(angle),
                ],
                [
                    0,
                    1,
                    0,
                ],
                [-sin(angle), 0, cos(angle)],
              
            ]
        )
    elif axis == "z":
        return Matrix(
            [
                [
                    cos(angle),
                    -sin(angle),
                    0,
                ],
                [
                    sin(angle),
                    cos(angle),
                    0,
                ],
                [
                    0,
                    0,
                    1,
                ],
            ]
        )
    else:
        raise ValueError("Invalid axis")


# a = transformation_matrix(symbols("q"), symbols("d"), symbols("a"), symbols("alpha"), symbols("offset"))
# a = translation_matrix(symbols("x"), symbols("y"), symbols("z"))
# print_latex(a)

# rx = rotation_matrix("x", symbols("Theta"))
# print_latex(Equality(symbols("R_x"), rx, evaluate=False))
# ry = rotation_matrix("y", symbols("Theta"))
# print_latex(Equality(symbols("R_y"), ry, evaluate=False))
# rz = rotation_matrix("z", symbols("Theta"))
# print_latex(Equality(symbols("R_z"), rz, evaluate=False))
# jacobian_matrix()
