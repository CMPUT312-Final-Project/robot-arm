from robot_kinematics import *


def test_pathing():
    start_joint_angle = JointAngles(0, 0, 0, 0, 0)
    end_joint_angle = JointAngles(
        theta1=-pi / 2,
        theta2=0,
        theta3=0,
        theta4=0,
        gripper=0,
    )
    print("Starting", start_joint_angle)
    print("Ending", end_joint_angle)
    angles = joint_interpolation(start_joint_angle, end_joint_angle)

    # for angle in angles:
    #     print(angle)


def test_inverse_kinematics():
    starting_j = JointAngles(0, 0, 0, 0, 0)
    x = [
        CartesianCoordinates(
            x=-0.5617379012604284, y=-12.726585837836798, z=3.3382126044630382
        ),
        CartesianCoordinates(x=-30.472, y=-3.850, z=20.240),
        CartesianCoordinates(x=4.636, y=-24.310, z=-3.642),
    ]
    print("Starting", starting_j)
    for i in range(len(x)):
        start = time.time()
        target = x[i]
        for j in range(100):
            j = inverse_kinematics(target, starting_j)
            obtained_ans = forward_kinematics(j)
            obtained_angles = j.radian_to_degree().to_np_array()

            # Assert obtained angles are in the range of -180 to 180
            assert np.all(np.abs(obtained_angles) <= 180)
            assert np.allclose(target.return_np(), obtained_ans.return_np(), atol=0.1)

        end = time.time()
        print("Time taken for", i, ":", end - start)



test_inverse_kinematics()
test_pathing()