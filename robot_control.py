from time import sleep

import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
from robot_kinematics import *

ADDR_PRESENT_POSITION = 36
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 10
ADDR_MX_PRESENT_POSITION = 50
ADDR_MX_PUNCH = 48
ADDR_MX_TORQUE = 14
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
PROTOCOL_VERSION = 1.0
DXL_IDS = [1, 2, 3, 4]
DXL_IDS_OFFSET = [150, 150, 150, 240]
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
DEVICENAME = "/dev/ttyACM0"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)
max_Values = [[0, 800], [170, 700], [200, 1024], [200, 700]]

def goal_pos_finder(centers, camera_transform_mtx, depth):
    '''
    Assuming the camera matrix contains the intrinsic camera parameters
    and the camera transformation matrix is a 4x4 homogeneous matrix
    which contains the values to pass from the camera frame to the world frame
    we calculate the positions of the red smarties in the world frame given
    their respective positions in the camera frame
    '''
    camera_mtx = np.array([[493.59710031,   0,         425.22959262],
                           [0,         612.7970329,  245.57518965],
                           [0,           0,           1        ]])

    camera_transform_mtx = np.array(camera_transform_mtx)
    centers = np.array(centers)  # Shape: (n, 2)

    camera_mtx_inv = np.linalg.inv(camera_mtx)
    centers_homogeneous = np.hstack([centers, np.ones((centers.shape[0], 1))])  # Shape: (n, 3)

    camera_3d_coords = depth * (camera_mtx_inv @ centers_homogeneous.T)  # Shape: (3, n)
    camera_3d_coords_homogeneous = np.vstack([camera_3d_coords, np.ones(camera_3d_coords.shape[1])])  # Shape: (4, n)

    # Transform to world frame
    world_coords_homogeneous = camera_transform_mtx @ camera_3d_coords_homogeneous  # Shape: (4, n)
    goal_pos = world_coords_homogeneous[:3, :].T  # Shape: (n, 3)

    return goal_pos.tolist()


def initial_pos_set(initial_pos=[0, 0, 0, 0],angle_type = "degrees",sleep_val= 0.1):


    if angle_type != "degrees":
        initial_pos = np.degrees(initial_pos)
    # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
    # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
    # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)

    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE, 0x100)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CW_ANGLE_LIMIT, max_Values[DXL_ID-1][0])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CCW_ANGLE_LIMIT, max_Values[DXL_ID-1][1])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 1)

    for DXL_ID in DXL_IDS:        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, round((initial_pos[DXL_ID-1] + DXL_IDS_OFFSET[DXL_ID-1]) * 1024/300))
        time.sleep(sleep_val)

            # if dxl_comm_result != COMM_SUCCESS:
                # print(f"Error writing to Motor {DXL_ID}: {packetHandler.getTxRxResult(dxl_comm_result)}")
        if dxl_error != 0:
            print(f"Error code {dxl_error} for Motor {DXL_ID} when writing position")

    return portHandler, packetHandler

def move_robot_to_point(P,portHandler,packetHandler):


    current_position = []
    for DXL_ID in DXL_IDS:
        data, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        current_position.append(data / (1024/300) - DXL_IDS_OFFSET[DXL_ID-1])

    T_0, _ = forward_kinematics(*current_position)

    points = upward_sphere_geodesic_with_linear_extension(T_0[:3, -1], P, num_points=10)
    Q = current_position
    for point in points:
        Q = numeric_inverse_function(point, Q)
        for DXL_ID in DXL_IDS:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                    portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, round((Q[DXL_ID-1] + DXL_IDS_OFFSET[DXL_ID-1]) * 1024/300)) #IS 4 Bytes necessary

