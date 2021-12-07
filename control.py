###############################################################################
# 	Author:			Qasim Khawaja
# 	Version:		1.0.0
# 	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#
# 	Description:	This is a library for controlling the robot arm.
###############################################################################

import asyncio
import time

import numpy as np
from pandas.core.frame import DataFrame
from pandas.core.resample import Resampler
from info_client import sendCartesian
from robot_kinematics import CartesianCoordinates, inverse_kinematics
import lss
import lss_const as lssc
from robot_kinematics import JointAngles, forward_kinematics
import sys, signal
from lss import LSS, getPositionMultipleServos
from util import motor_angles_to_joint_angles
from LSS_movement import move_to_angle
from threading import Thread
import pandas as pd
import matplotlib.pyplot as plt

# Global dictionary to store sensor data. Updated by sensor thread
servo_positions = {
    '1': 0,
    '2': 0,
    '3': 0,
    '4': 0,
}

def _handler(signal, frame):
    '''
        To handle Ctrl+C and safely close the program
    '''
    print("\nYou pressed Ctrl+C!")
    plt.close('all')
    try:
        lss.closeBus()
    except:
        print("Error closing bus")
    sys.exit(0)

signal.signal(signal.SIGINT, _handler)

try:
    # Initialize the LSS and various settings
    PORT = "/dev/tty.usbserial-A10KM8FE"
    lss.initBus(PORT, lssc.LSS_DefaultBaud)
    base = LSS(1)
    shoulder = LSS(2)
    elbow = LSS(3)
    wrist = LSS(4)
    gripper = LSS(5)
    allMotors = LSS(254)
    shoulder.setGyre(lssc.LSS_GyreCounterClockwise)
    base.setOriginOffset(-1799)
    elbow.setOriginOffset(0)
    # Settings from LSS_movement.py
    allMotors.setAngularHoldingStiffness(-1)
    # allMotors.setMotionControlEnabled(0)
    allMotors.setMaxSpeed(20)
    # allMotors.setAngularAcceleration(0)
    # allMotors.setAngularDeceleration(0)
    # allMotors.setFilterPositionCount(20)
    # elbow.setMotionControlEnabled(0)
    event_loop_a = asyncio.new_event_loop()
except:
    print("Error: Could not connect to LSS")

def path(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS):
    '''
        Move the arm on a path and send interpolated data to the server
    '''
    # 2 pre-set Joint Angles
    allMotors.setMaxSpeed(20)
    move_to_angle(base, shoulder, elbow, wrist, gripper, JointAngles(0, 0, 0, 0, 0))
    j2 = JointAngles(theta1=338, theta2=-661, theta3=259, theta4=401, gripper=0)
    
    Thread(target=sensorFromFile, daemon=True).start()
    move_to_angle(base, shoulder, elbow, wrist, gripper, j2, False)
    while True:
        # Wait
        input("Waiting")

def track_arm_position(base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS):
    '''
        Track the arm position and send it to the server
    '''
    
    # Start thread to update sensor data
    Thread(target=update_sensor_data, daemon=True).start()
    # Start thread to send sensor data to server
    Thread(target=sensor, args=(base, shoulder, elbow, wrist, gripper), daemon=True).start()
    while True:
        input_key = input("Select mode (h,l):")
        if input_key == "h":
            base.hold()
            shoulder.hold()
            elbow.hold()
            wrist.hold()
            gripper.hold()
            allMotors.hold()
        else:
            base.limp()
            shoulder.limp()
            elbow.limp()
            wrist.limp()
            gripper.limp()
            allMotors.limp()
        
def sensorFromFile():
    '''
        Thread to read sensor data from a file and send it to the server
    '''
    asyncio.set_event_loop(event_loop_a)
    pos_list =  readFromFile()
    i = 1
    pos_list = np.array(pos_list)
    while i<len(pos_list):
        i += 1
        coordinates = CartesianCoordinates(pos_list[i][0], pos_list[i][1], pos_list[i][2])
        xr = pos_list[i][3]
        yr = pos_list[i][4]
        asyncio.get_event_loop().run_until_complete(sendCartesian(coordinates, xr, yr))

def update_sensor_data():
    '''
        Uses a modified query to update all servo positions
    '''
    while True:
        global servo_positions
        start = time.time()
        # Using a modified query to get all servo positions at the same time
        data = getPositionMultipleServos(['1','2','3','4'])
        end = time.time()
        if data:
            servo_positions = data
        print(f"Time taken: {end-start}")
        
        # This method waits for each servo position
        # servo_positions["1"]=str(base.getPosition())
        # servo_positions["2"]=str(shoulder.getPosition())
        # servo_positions["3"]=str(elbow.getPosition())
        # servo_positions["4"]=str(wrist.getPosition())

def sensor(base, shoulder, elbow, wrist, gripper):
    '''
    Thread to read sensor data and send it to the server
    '''
    asyncio.set_event_loop(event_loop_a)
    pos = JointAngles(0, 0, 0, 0, 0).degree_to_radian()
    global servo_positions
    while True:
        posRead = JointAngles(
                str(servo_positions["1"]),
                str(servo_positions["2"]),
                str(servo_positions["3"]),
                str(servo_positions["4"]),
                '0',
            )
        if posRead.is_valid():
            pos = posRead.from_motor_degrees()
            pos: JointAngles = pos.degree_to_radian()
            # May want to use: 
            # if pos.has_changed(lastPos, 0.001):
            coordinates: CartesianCoordinates = forward_kinematics(pos)
            xr = pos.theta2+pos.theta3+pos.theta4 #+ 19.3 * np.pi/180 # Cube is 19.3 degrees from the base
            yr = pos.theta1
            start = time.time()
            asyncio.get_event_loop().run_until_complete(sendCartesian(coordinates, xr, yr))
            end = time.time()
            print(f"Server Time taken: {end-start}")

def saveToFile(coordinates: CartesianCoordinates, xr, yr, seconds):
    '''
        Save the sensor data to a file. (Used for predefined paths)
    '''
    with open("data.txt", "a") as f:
        f.write(f"{seconds},{coordinates.x},{coordinates.y},{coordinates.z},{xr},{yr}\n")

def readFromFile():
    data = []
    with open("data.txt", "r") as f:
        lines = f.readlines()
        data = [line.split(",") for line in lines]
    transformed_data = []
    for point in data:
        coordinates = CartesianCoordinates(float(point[1]), float(point[2]), float(point[3]))
        xr = float(point[4])
        yr = float(point[5])
        timestamp = float(point[0])
        transformed_data.append((timestamp,*coordinates.return_np(), xr, yr))
    df = pd.DataFrame(transformed_data, columns=['timestamp','x', 'y', 'z', 'xr', 'yr'])
    # convert timestamp to DateTime index
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
    df.set_index('timestamp', inplace=True)
    
    # Interpolate to fixed time step
    timestep = 1.2
    upsample: Resampler = df.resample(f'{timestep}S').mean()
    interpolate: DataFrame = upsample.interpolate(method='cubicspline')
    # interpolate.plot()
    # plt.show()
    # print(interpolate)
    return interpolate

def move_arm(
    target: CartesianCoordinates,
    base: LSS,
    shoulder: LSS,
    elbow: LSS,
    wrist: LSS,
    gripper: LSS,
    allMotors: LSS,
    divisions=1,
):
    current_theta: JointAngles = motor_angles_to_joint_angles(
        base, shoulder, elbow, wrist, gripper
    )
    current_pos: CartesianCoordinates = forward_kinematics(current_theta)

    delta_pos = (target - current_pos) / divisions

    for i in range(divisions):
        new_pos = current_pos + delta_pos
        move_angles: JointAngles = inverse_kinematics(new_pos, current_theta)
        move_angles = move_angles.radian_to_degree().to_motor_degrees()

        a = move_to_angle(base, shoulder, elbow, wrist, gripper, move_angles)
        current_theta = motor_angles_to_joint_angles(
            base, shoulder, elbow, wrist, gripper
        )
        current_pos = forward_kinematics(current_theta)


def control_with_keys(
    base: LSS, shoulder: LSS, elbow: LSS, wrist: LSS, gripper: LSS, allMotors: LSS
):

    while True:
        initial_pos = forward_kinematics(
            motor_angles_to_joint_angles(base, shoulder, elbow, wrist, gripper)
        )
        input_key = input("Select")
        if input_key == "w":
            move_arm(
                initial_pos + CartesianCoordinates(0, 0, 2),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "s":
            move_arm(
                initial_pos - CartesianCoordinates(0, 0, 2),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "a":
            move_arm(
                initial_pos + CartesianCoordinates(2, 0, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "d":
            move_arm(
                initial_pos - CartesianCoordinates(2, 0, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "r":
             move_arm(
                initial_pos + CartesianCoordinates(0, 2, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "l":
            move_arm(
                initial_pos - CartesianCoordinates(0, 2, 0),
                base,
                shoulder,
                elbow,
                wrist,
                gripper,
                allMotors,
            )
        elif input_key == "q":
            break



if __name__ == "__main__":
    # move_arm(CartesianCoordinates(x=-3.505244735627988, y=-7.872908577754623, z=10.5552216211064342), base, shoulder, elbow, wrist, gripper, allMotors)
    track_arm_position(base, shoulder, elbow, wrist, gripper, allMotors)
    # path(base, shoulder, elbow, wrist, gripper, allMotors)
    # control_with_keys(base, shoulder, elbow, wrist, gripper, allMotors)
    # readFromFile()
