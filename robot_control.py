from time import sleep

import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

from numpy.core.function_base import linspace

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
DXL_IDS_OFFSET = [150, 150, 150, 150]
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
DEVICENAME = "/dev/ttyACM0"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
max_Values = [[0, 800], [170, 700], [200, 1024], [200, 700]]


def goal_pos_finder(pixel_coords, camera_position, plane_z):
    """
    Calculate the world frame positions of points on a plane given image plane coordinates.
    
    :param pixel_coords: List of 2D coordinates (x, y) in pixels.
    :param camera_position: List or array [x, y, z] of the camera in the world frame.
    :param plane_z: Z-coordinate of the plane in the world frame.
    :param camera_matrix: 3x3 intrinsic camera matrix.
    :return: List of world frame coordinates for the input pixel points.
    """
    camera_matrix = np.array([[493.59710031,   0,         425.22959262],
                      [0,         612.7970329,  245.57518965],
                      [0,           0,           1        ]])

    world_positions = []
    
    # Camera position in world frame
    C = np.array(camera_position)
    
    # Invert the camera matrix
    K_inv = np.linalg.inv(camera_matrix)
    
    
    # Convert pixel to normalized camera coordinates
    u = np.array([pixel_coords[0], pixel_coords[1], 1])  # Homogeneous pixel coordinates
    x_c = K_inv @ u  # Camera coordinates (direction vector)
    
    # Compute direction vector from camera to the point
    direction = x_c / np.linalg.norm(x_c)  # Normalize the direction vector
    
    # Calculate t for intersection with the plane
    t = (plane_z - C[2]) / direction[2]
    
    # Calculate world position
    P_world = C + t * direction
    world_positions.append(P_world)
    
    return P_world


def initial_pos_set(initial_pos=[0, 0, 0, 0],angle_type = "degrees",sleep_val= 0.1):
    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

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

def move_robot_to_point(goal_point,current_position,portHandler,packetHandler,sleep_val=0.01):
    print("new_point")

    # current_position = []
    # for DXL_ID in DXL_IDS:
    #     data, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    #     current_position.append(data / (1024/300) - DXL_IDS_OFFSET[DXL_ID-1])

    T_0, _ = forward_kinematics(*current_position)
    if goal_point[-1] <-100:
        points = upward_sphere_geodesic_with_linear_extension(T_0[:3, -1], goal_point, num_points=10)
    else:
        points = np.linspace(T_0[:3, -1], goal_point, num=50)
    print(goal_point)
    Q = current_position
    for point in points:
        Q = numeric_inverse_function(point, Q)
        for DXL_ID in DXL_IDS:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                    portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, round((np.degrees(Q[DXL_ID-1]) + DXL_IDS_OFFSET[DXL_ID-1]) * 1024/300)) #IS 4 Bytes necessary
            time.sleep(sleep_val)
        t_0,_ =forward_kinematics(*Q)
        print(t_0[:3,-1])
    return Q

def move_robot_to_point2(goal_point,current_position,portHandler,packetHandler,sleep_val=0.01):


    # current_position = []
    # for DXL_ID in DXL_IDS:
    #     data, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    #     current_position.append(data / (1024/300) - DXL_IDS_OFFSET[DXL_ID-1])

    Q = numeric_inverse_function(goal_point,current_position)
    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_GOAL_POSITION,
            round((np.degrees(-1) + DXL_IDS_OFFSET[DXL_ID - 1]) * 1024 / 300))  # IS 4 Bytes necessary
        time.sleep(sleep_val)

    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, DXL_ID, ADDR_MX_GOAL_POSITION,
            round((np.degrees(Q[DXL_ID - 1]) + DXL_IDS_OFFSET[DXL_ID - 1]) * 1024 / 300))  # IS 4 Bytes necessary
        time.sleep(sleep_val)

    return Q


def lift_arm(current_angles,portHandler,packetHandler,sleep_val=0.01):

    goal_angles = np.array(current_angles)+ np.array([0,np.pi/3,0,0])


    points = np.linspace(current_angles, goal_angles, num=10)
    Q = current_angles
    for point in points:
        DXL_ID = 2
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE, 0x200)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, round((np.degrees(Q[DXL_ID-1]) + DXL_IDS_OFFSET[DXL_ID-1]) * 1024/300)) #IS 4 Bytes necessary
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE, 0x100)
    time.sleep(sleep_val)

    return Q
