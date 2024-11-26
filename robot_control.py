from time import sleep

import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time


def move_robot(pos_list_q):
    '''Moves the robot through a series of joint positions (pos_list_q).
Main Steps:numer
Communication Setup:
Uses Dynamixel SDK to communicate with motors via a serial port.
Sets baud rate and initializes motors.
Disable Torque:
Ensures motors are not under load before setting positions.
Joint Controls:
Converts joint angles to Dynamixel-compatible values using offsets and scaling.
Iterates through the steps of joint configurations and sends commands to the motors.
Movement:
Each joint position is written using the write4ByteTxRx method.
Includes delays (time.sleep) to allow motors to reach their target positions.
Disable Torque After Movement:
Ensures motors are not under torque when idle.
Close Port:
Ends communication with motors.'''
    
    ADDR_MX_TORQUE_ENABLE = 24
    ADDR_MX_GOAL_POSITION = 30
    PROTOCOL_VERSION = 1.0
    DEVICENAME = "/dev/ttyACM0"
    BAUDRATE = 1000000
    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    max_Values = [[0,800],[170,700],[200,1024],[200,700]]
    DXL_IDS = [1, 2, 3, 4]
    DXL_IDS_OFFSET = [148, 57, 151, 240]

    for DXL_ID in DXL_IDS:        
        print("Disabling torque...")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

    controls_Q = [[(np.rad2deg(val) + DXL_IDS_OFFSET[i]) * 1024/300 for val in row] for i, row in enumerate(pos_list_q)]

    for step in range(len(controls_Q[0])):
        for idx, DXL_ID in enumerate(DXL_IDS):

            goal_pos = controls_Q[idx][step]
                            
            # Write the goal position to the motor
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_pos
            )
            # if dxl_comm_result != COMM_SUCCESS:
                # print(f"Error writing to Motor {DXL_ID}: {packetHandler.getTxRxResult(dxl_comm_result)}")
            if dxl_error != 0:
                print(f"Error code {dxl_error} for Motor {DXL_ID} when writing position")
        
        # Optional: Wait for a short duration to allow motors to reach the position
        # Adjust the sleep time as necessary based on motor speed and movement requirements
        if step == 0:
            time.sleep(1)
        else:
            time.sleep(0.01)

    for DXL_ID in DXL_IDS:        
        print("Disabling torque...")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

    print("Movement sequence completed successfully.")

    portHandler.closePort()


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

    # ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
    # ADDR_MX_CW_COMPLIANCE_SLOPE = 28
    # ADDR_MX_CCW_COMPLIANCE_SLOPE = 10
    # ADDR_MX_PRESENT_POSITION = 50
    # ADDR_MX_PUNCH = 48
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
    max_Values = [[0,800],[170,700],[200,1024],[200,700]]
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
    portHandler.closePort()


def move_robot_to_next_point(P, angle_type = "degrees",sleep_val= 0.1):

    # ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
    # ADDR_MX_CW_COMPLIANCE_SLOPE = 28
    # ADDR_MX_CCW_COMPLIANCE_SLOPE = 10
    # ADDR_MX_PRESENT_POSITION = 50
    # ADDR_MX_PUNCH = 48
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
    max_Values = [[0,800],[170,700],[200,1024],[200,700]]
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