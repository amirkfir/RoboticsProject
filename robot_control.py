import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time


def move_robot(pos_list_q):
    
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


def goal_pos_finder(camera_mtx, centers, camera_transform_mtx, depth=0.286):
    '''
    Assuming the camera matrix contains the intrinsic camera parameters
    and the camera transformation matrix is a 4x4 homogeneous matrix
    which contains the values to pass from the camera frame to the world frame
    we calculate the positions of the red smarties in the world frame given 
    their respective positions in the camera frame
    '''

    camera_mtx = np.array(camera_mtx)
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


def initial_pos_set(initial_pos=[0, 0, 0, 0],angle_type = "degrees"):

    # ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
    # ADDR_MX_CW_COMPLIANCE_SLOPE = 28
    # ADDR_MX_CCW_COMPLIANCE_SLOPE = 10
    # ADDR_MX_PRESENT_POSITION = 50
    # ADDR_MX_PUNCH = 48
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
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CW_ANGLE_LIMIT, max_Values[DXL_ID-1][0])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CCW_ANGLE_LIMIT, max_Values[DXL_ID-1][1])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 1)

    for DXL_ID in DXL_IDS:        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, round((initial_pos[DXL_ID-1] + DXL_IDS_OFFSET[DXL_ID-1]) * 1024/300))
        time.sleep(0.1)
        print(DXL_ID)

            # if dxl_comm_result != COMM_SUCCESS:
                # print(f"Error writing to Motor {DXL_ID}: {packetHandler.getTxRxResult(dxl_comm_result)}")
        if dxl_error != 0:
            print(f"Error code {dxl_error} for Motor {DXL_ID} when writing position")

    # for DXL_ID in DXL_IDS:
        # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)



# centros = [[1, 1], [2, 2], [3, 3]]
# matriz_camera = [[800, 0, 640], [0, 800, 360], [0, 0, 1]]
# T50 = [[1/2, np.sqrt(3)/2, 0, 0], [-1/2, np.sqrt(3)/2, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
# goal_pos = goal_pos_finder(matriz_camera, centros, T50)
# print(goal_pos)




# Q = [[0.2102, 0.2071, 0.1978, 0.1827, 0.1620, 0.1363, 0.1063, 0.0728, 0.0370, 0.0000
# , -0.0370, -0.0728, -0.1063, -0.1363, -0.1620, -0.1827, -0.1978, -0.2071, -0.2102
# , -0.2071, -0.1978, -0.1827, -0.1620, -0.1363, -0.1063, -0.0728, -0.0370, -0.0000
# , 0.0370, 0.0728, 0.1063, 0.1363, 0.1620, 0.1827, 0.1978, 0.2071, 0.2102],
# [-0.2397, -0.1805, -0.1233, -0.0698, -0.0214, 0.0204, 0.0544, 0.0795, 0.0950
# , 0.1002, 0.0950, 0.0795, 0.0544, 0.0204, -0.0214, -0.0698, -0.1233, -0.1805
# , -0.2397, -0.2994, -0.3579, -0.4134, -0.4643, -0.5087, -0.5452, -0.5724, -0.5892
# , -0.5949, -0.5892, -0.5724, -0.5452, -0.5087, -0.4643, -0.4134, -0.3579, -0.2994
# , -0.2397],
# [1.6699, 1.6242, 1.5789, 1.5354, 1.4952, 1.4597, 1.4304, 1.4084, 1.3949, 1.3903
# , 1.3949, 1.4084, 1.4304, 1.4597, 1.4952, 1.5354, 1.5789, 1.6242, 1.6699, 1.7146
# , 1.7571, 1.7963, 1.8311, 1.8608, 1.8846, 1.9019, 1.9125, 1.9160, 1.9125, 1.9019
# , 1.8846, 1.8608, 1.8311, 1.7963, 1.7571, 1.7146, 1.6699],
# [-1.4302, -1.4437, -1.4556, -1.4656, -1.4737, -1.4801, -1.4848, -1.4880, -1.4898
# , -1.4904, -1.4898, -1.4880, -1.4848, -1.4801, -1.4737, -1.4656, -1.4556, -1.4437
# , -1.4302, -1.4152, -1.3992, -1.3829, -1.3669, -1.3521, -1.3393, -1.3295, -1.3233
# , -1.3212, -1.3233, -1.3295, -1.3393, -1.3521, -1.3669, -1.3829, -1.3992, -1.4152
# , -1.4302]]

# DXL_IDS_OFFSET = [148, 147, 151, 240]
# controls_Q = [[(np.rad2deg(val) + DXL_IDS_OFFSET[i]) * 1024/300 for val in row] for i, row in enumerate(Q)]
# colors = ['black', 'blue', 'red', 'orange']
# max_Values = [[0,800],[170,710],[200,1024],[190,700]]


# fig, axes = plt.subplots(1, 2, figsize=(10, 6))
# axes = axes.flatten()

# for i, row in enumerate(controls_Q):
#     axes[0].plot(row, label=f'Controls Joint {i+1}', color=colors[i])
#     axes[0].axhline(y=max_Values[i][0], color=colors[i], label=f'min value for joint {i+1}')
#     axes[0].axhline(y=max_Values[i][1], color=colors[i], label=f'max value for joint {i+1}')
#     axes[0].set_title('Values for controls of joints')
#     axes[0].set_ylabel('Input in decimal')
#     axes[0].legend()

# axes[1].set_title('Angles for Joint 2')
# axes[1].plot(np.rad2deg(Q[1]) + DXL_IDS_OFFSET[1])
# axes[1].axhline(y=max_Values[1][0] * 300 /1024)
# axes[1].axhline(y=max_Values[1][1] * 300 /1024)

# plt.tight_layout()
# plt.show()
# plt.title('Values for controls of joints')
# plt.ylabel('Input in decimal')
# plt.legend()
# plt.show()

# for DXL_ID in DXL_IDS:
#     for i in range (max_Values[DXL_ID-1][0],max_Values[DXL_ID-1][1]):
#         dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)
#         dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION,
#                                                                 i)
#         dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)




# try:
#     c = cv2.VideoCapture(2)
# except:
#     print("Cam 2 is invalid.")

# if c is not None:
#     if c.grab():
#         flag, frame = c.retrieve()
#         plt.imshow(frame)
#         plt.show()
# portHandler.closePort()


# 450 --> 0x0200
# 179 --> 0x0263

