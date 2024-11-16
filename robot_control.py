import dynamixel_sdk as dxl
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time


def conversion(angle_rad):
    if angle_rad < 0:
        angle_rad = 2 * np.pi + angle_rad

    angle_degrees = np.rad2deg(angle_rad) % 360
    m = 1024/300 # conversion factor taken from robot

    return int(angle_degrees * m)

Q = [[0.2102, 0.2071, 0.1978, 0.1827, 0.1620, 0.1363, 0.1063, 0.0728, 0.0370, 0.0000
, -0.0370, -0.0728, -0.1063, -0.1363, -0.1620, -0.1827, -0.1978, -0.2071, -0.2102
, -0.2071, -0.1978, -0.1827, -0.1620, -0.1363, -0.1063, -0.0728, -0.0370, -0.0000
, 0.0370, 0.0728, 0.1063, 0.1363, 0.1620, 0.1827, 0.1978, 0.2071, 0.2102],
[-0.2397, -0.1805, -0.1233, -0.0698, -0.0214, 0.0204, 0.0544, 0.0795, 0.0950
, 0.1002, 0.0950, 0.0795, 0.0544, 0.0204, -0.0214, -0.0698, -0.1233, -0.1805
, -0.2397, -0.2994, -0.3579, -0.4134, -0.4643, -0.5087, -0.5452, -0.5724, -0.5892
, -0.5949, -0.5892, -0.5724, -0.5452, -0.5087, -0.4643, -0.4134, -0.3579, -0.2994
, -0.2397],
[1.6699, 1.6242, 1.5789, 1.5354, 1.4952, 1.4597, 1.4304, 1.4084, 1.3949, 1.3903
, 1.3949, 1.4084, 1.4304, 1.4597, 1.4952, 1.5354, 1.5789, 1.6242, 1.6699, 1.7146
, 1.7571, 1.7963, 1.8311, 1.8608, 1.8846, 1.9019, 1.9125, 1.9160, 1.9125, 1.9019
, 1.8846, 1.8608, 1.8311, 1.7963, 1.7571, 1.7146, 1.6699],
[-1.4302, -1.4437, -1.4556, -1.4656, -1.4737, -1.4801, -1.4848, -1.4880, -1.4898
, -1.4904, -1.4898, -1.4880, -1.4848, -1.4801, -1.4737, -1.4656, -1.4556, -1.4437
, -1.4302, -1.4152, -1.3992, -1.3829, -1.3669, -1.3521, -1.3393, -1.3295, -1.3233
, -1.3212, -1.3233, -1.3295, -1.3393, -1.3521, -1.3669, -1.3829, -1.3992, -1.4152
, -1.4302]]

DXL_IDS_OFFSET = [148, 147, 151, 240]
controls_Q = [[(np.rad2deg(val) + DXL_IDS_OFFSET[i]) * 1024/300 for val in row] for i, row in enumerate(Q)]
colors = ['black', 'blue', 'red', 'orange']
max_Values = [[0,800],[170,710],[200,1024],[190,700]]


fig, axes = plt.subplots(1, 2, figsize=(10, 6))
axes = axes.flatten()

for i, row in enumerate(controls_Q):
    axes[0].plot(row, label=f'Controls Joint {i+1}', color=colors[i])
    axes[0].axhline(y=max_Values[i][0], color=colors[i], label=f'min value for joint {i+1}')
    axes[0].axhline(y=max_Values[i][1], color=colors[i], label=f'max value for joint {i+1}')
    axes[0].set_title('Values for controls of joints')
    axes[0].set_ylabel('Input in decimal')
    axes[0].legend()

axes[1].set_title('Angles for Joint 2')
axes[1].plot(np.rad2deg(Q[1]) + DXL_IDS_OFFSET[1])
axes[1].axhline(y=max_Values[1][0] * 300 /1024)
axes[1].axhline(y=max_Values[1][1] * 300 /1024)

plt.tight_layout()
plt.show()
# plt.title('Values for controls of joints')
# plt.ylabel('Input in decimal')
# plt.legend()
# plt.show()

def move_robot(pos_list_q):
    
    ADDR_MX_TORQUE_ENABLE = 24
    ADDR_MX_CW_COMPLIANCE_MARGIN = 26
    ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
    ADDR_MX_CW_COMPLIANCE_SLOPE = 28
    ADDR_MX_CCW_COMPLIANCE_SLOPE = 10
    ADDR_MX_GOAL_POSITION = 30
    ADDR_MX_MOVING_SPEED = 32
    ADDR_MX_PRESENT_POSITION = 50
    ADDR_MX_PUNCH = 48
    PROTOCOL_VERSION = 1.0
    DXL_IDS = [1, 2, 3, 4]
    DXL_IDS_OFFSET = [148, 57, 151, 240]
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

    for DXL_ID in DXL_IDS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CW_ANGLE_LIMIT, max_Values[DXL_ID-1][0])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CCW_ANGLE_LIMIT, max_Values[DXL_ID-1][1])
        # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
        # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
        # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 1)

    num_steps = min(len(pos_list_q[0]), len(pos_list_q[1]), len(pos_list_q[2]), len(pos_list_q[3]))

    for DXL_ID in DXL_IDS:        
        print("Disabling torque...")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, 0)

    for step in range(num_steps):
        for idx, DXL_ID in enumerate(DXL_IDS):
            
            angle_rad = pos_list_q[idx][step]
            
            
            goal_pos = conversion(angle_rad + np.deg2rad(DXL_IDS_OFFSET[idx]) + np.pi/2)

                            
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

    return 



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

