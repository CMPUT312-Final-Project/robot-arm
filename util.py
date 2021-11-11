from robot_kinematics import JointAngles
from lss import LSS
def motor_angles_to_joint_angles(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS):
    return JointAngles(
        base.getPosition(),
        shoulder.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        gripper.getPosition(),
    ).from_motor_degrees().degree_to_radian()