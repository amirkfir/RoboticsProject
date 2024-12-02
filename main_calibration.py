import cv2
import time
import os
import numpy as np
from robot_kinematics import inverse_kinematics, forward_kinematics, inverse_kinematics2,numeric_inverse_function
from robot_control import *
from ImageProcessing import calibration, get_color_coordinates
from camera_calibration import camera_calibration, undistort
import matplotlib.pyplot as plt


def save_frame(directory, frame, count):
    """
    Saves a frame to the specified directory with a numbered filename.
    """
    filename = os.path.join(directory, f"image_{count:03d}.jpg")
    cv2.imwrite(filename, frame)
    print(f"Saved: {filename}")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

#     first_position = np.radians([0, -0, -90, -90])  # np.radians([0, -1, -1, -90]) #np.radians([0, -30, -40, -90])
#     portHandler, packetHandler = initial_pos_set(initial_pos=first_position, angle_type='radians', sleep_val=2)
#     # c = None
#     # try:
#     #     c = cv2.VideoCapture(2)
#     # except:
#     #     print("Cam 2 is invalid.")
#     #
#     # if c is not None:
#     # c.read()
#     # Ensure the directory for storing images exists
    output_dir = "calibration_images"
#     os.makedirs(output_dir, exist_ok=True)
#     c = None
#     try:
#         c = cv2.VideoCapture(2)
#
#     except:
#         print("Cam 2 is invalid.")
#
#     if c is not None:
#         flag, frame = c.read()
#         if flag:
#             plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#             plt.show()
#         else:
#             print("Failed to get initial image")
#         # input("set calibration setup and press any key to start")
#         # for _ in range (20):
#         #     flag, frame = c.read()
#
#
# #     for center in centers:
# #         plt.scatter(center[0],center[1])
# #     plt.show()
# #
# # input("Prepare the chessboard and press any key")
# # images = []
# # for _ in range(5):
# #     for __ in range(20):
# #         flag, frame = c.read()
# #     if flag:
# #         images.append(frame)
# #         plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
# #         plt.show()
# #     input("Change chessboard position")
# #
# # c.release()
# # cam_mtx = camera_calibration(images=images)
#
#         input("Prepare the chessboard and press any key")
#         count = 0
#
#         for _ in range(10):
#             for __ in range(10):  # Capture multiple frames to allow stabilization
#                 flag, frame = c.read()
#             if flag:
#                 plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#                 plt.show()
#                 input("Adjust chessboard position")
#
#             for __ in range(10):  # Capture multiple frames to allow stabilization
#                 flag, frame = c.read()
#             if flag:
#                 save_frame(output_dir, frame, count)
#                 count += 1
#
#
#             else:
#                 print("Failed to capture frame.")
#             input("Change chessboard position")
#
#         c.release()
#
#     portHandler.closePort()
    image_files = [os.path.join(output_dir, f) for f in os.listdir(output_dir) if f.endswith('.jpg')]
    images = [cv2.imread(img_file) for img_file in image_files]
    cam_mtx = camera_calibration(images=images)
    print(cam_mtx)


        

        

        









        


