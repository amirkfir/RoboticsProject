import cv2
import time
import os
import numpy as np
from robot_kinematics import inverse_kinematics, forward_kinematics, inverse_kinematics2,numeric_inverse_function
from robot_control import *
from ImageProcessing import calibration, get_color_coordinates
from camera_calibration import camera_calibration, undistort
import matplotlib.pyplot as plt


def main():

    c = None

    try:
        # Set port number for robot's camera
        port_number = 2
        print(f'Using port number: {port_number}')
        
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

        # Calibration step
        print("Starting chessboard calibration. Prepare the chessboard.")
        chessboard_images = []
        for _ in range(10):
            for __ in range(20):
                flag, frame = c.read()
            if flag:
                plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                plt.show()
                input("Adjust chessboard position and press enter.")
                chessboard_images.append(frame)

        if chessboard_images:
            new_cam_mtx, cam_mtx, dist, roi = camera_calibration(images=chessboard_images)

            print("New matrix obtained:", new_cam_mtx)
            print("Camera matrix obtained:", cam_mtx)
            print("Distortion parameters obtained:", dist)
            print("Region of interest obtained:", roi)

        else:
            print("Failed to capture chessboard images. Exiting.")
            c.release()
            return


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    first_position = np.radians([0, -1, -90, -1])  # Set the initial position

    portHandler, packetHandler = initial_pos_set(initial_pos=first_position, angle_type='radians', sleep_val=2)
    
    main()

    portHandler.closePort()


        

        

        









        


