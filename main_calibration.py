import cv2
import time
import os
import numpy as np
from robot_kinematics import inverse_kinematics, forward_kinematics, inverse_kinematics2,numeric_inverse_function
from robot_control import *
from ImageProcessing import calibration, get_color_coordinates
from camera_calibration import camera_calibration, undistort
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # Put the robot in the desired position for calibration
    q1, q2, q3, q4 = 0, -30, -65, -83
    move_robot_to_point([0, -30, -65, -83],portHandler, packetHandler)
    c = None
    try:
        # Set port number for robot's camera
        port_number = 4
        c = cv2.VideoCapture(port_number)
    except:
        print(f"Cam {port_number} is invalid")

    if c is not None:
        
        # Get robot's camera frame to adjust
        flag, frame = c.read()

        if flag:
            plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            plt.show()

        else:
            print("Failed to get initial image")

        print("Starting single smartie calibration")
        input("Put a single smartie in frame and press enter")

        for _ in range(20):
            flag, frame = c.read()

        if flag:
            plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            plt.show()

        input("Final adjustements for single smartie and press enter")

        for _ in range(20):
            flag, frame = c.read()

        if flag:
            frame = undistort(frame)
            color, radius = calibration(frame, 1)

        for _ in range(20):
            flag, frame = c.read()

        if flag:
            frame = undistort(frame)
            centers = get_color_coordinates(frame,color,radius,1)
            plt.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            for center in centers:
                plt.scatter(center[0],center[1])
            plt.show()

        # Calculate the forward kinematics to get the camera's position wrt world frame
        _, T50 = forward_kinematics(q1, q2, q3, q4)
        T50 = np.array(T50)

        # Get the goal positions (x, y, z) coordinates wrt world frame for red smarties
        # We know the plane of the table where the smarties sit

        z = -10 # Table plane
        z_c = T50[3, 3] # camera position

        goal_positions = goal_pos_finder(centers, T50, z_c - z)
        for pos in goal_positions:
            print(pos)
            


        

        

        









        

