# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import cv2
import numpy as np
from robot_kinematics import inverse_kinematics, forward_kinematics
from robot_control import move_robot, initial_pos_set, goal_pos_finder
from ImageProcessing import calibration, get_color_coardinates
from camera_calibration import camera_calibration
import matplotlib.pyplot as plt

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    c = None
    try:
        c = cv2.VideoCapture(2)
    except:
        print("Cam 2 is invalid.")

    if c is not None:
        c.read()
        input("set calibration setup and press any key to start")
        for _ in range (10):
            flag, frame = c.read()
        if flag:
            color,radius = calibration(frame,1)

        input("press any key to start")
        for _ in range (10):
            flag, frame = c.read()
        if flag:
            centers = get_color_coardinates(frame,color,radius,1)
            plt.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            for center in centers:
                plt.scatter(center[0],center[1])
            plt.show()
        
        input("Prepare the chessboard and press any key")
        images = []
        for _ in range(20):
            flag, frame = c.read()
            if flag:
                images.append(frame)
            input("Change chessboard position")

        c.release()
        cam_mtx = camera_calibration(images=images)
        q1_i, q2_i, q3_i, q4_i = 0, 0, 0, 0
        initial_pos=[q1_i, q2_i, q3_i, q4_i] # in degrees
        initial_pos_set(initial_pos)
        T04, T05 = forward_kinematics(q1_i, q2_i, q3_i, q4_i)
        goal_pos = goal_pos_finder(cam_mtx, centers, T05)
        Q = []
        for pos in goal_pos:
            Q.append(inverse_kinematics(pos))
        Q = np.array(Q)
        pos_list_q = []
        for column in Q.T:
            pos_list_q.append(column.tolist())
        move_robot(pos_list_q)

        
        




    # initial_pos_set()
    # cam_mtx = camera_calibration()

    

#See PyCharm help at https://www.jetbrains.com/help/pycharm/
# if __name__ == '__main__':
#     Q = [[0.2102, 0.2071, 0.1978, 0.1827, 0.1620, 0.1363, 0.1063, 0.0728, 0.0370, 0.0000
#    , -0.0370, -0.0728, -0.1063, -0.1363, -0.1620, -0.1827, -0.1978, -0.2071, -0.2102
#    , -0.2071, -0.1978, -0.1827, -0.1620, -0.1363, -0.1063, -0.0728, -0.0370, -0.0000
#    , 0.0370, 0.0728, 0.1063, 0.1363, 0.1620, 0.1827, 0.1978, 0.2071, 0.2102],
#     [-0.2397, -0.1805, -0.1233, -0.0698, -0.0214, 0.0204, 0.0544, 0.0795, 0.0950
#    , 0.1002, 0.0950, 0.0795, 0.0544, 0.0204, -0.0214, -0.0698, -0.1233, -0.1805
#    , -0.2397, -0.2994, -0.3579, -0.4134, -0.4643, -0.5087, -0.5452, -0.5724, -0.5892
#    , -0.5949, -0.5892, -0.5724, -0.5452, -0.5087, -0.4643, -0.4134, -0.3579, -0.2994
#    , -0.2397],
#     [1.6699, 1.6242, 1.5789, 1.5354, 1.4952, 1.4597, 1.4304, 1.4084, 1.3949, 1.3903
#    , 1.3949, 1.4084, 1.4304, 1.4597, 1.4952, 1.5354, 1.5789, 1.6242, 1.6699, 1.7146
#    , 1.7571, 1.7963, 1.8311, 1.8608, 1.8846, 1.9019, 1.9125, 1.9160, 1.9125, 1.9019
#    , 1.8846, 1.8608, 1.8311, 1.7963, 1.7571, 1.7146, 1.6699],
#     [-1.4302, -1.4437, -1.4556, -1.4656, -1.4737, -1.4801, -1.4848, -1.4880, -1.4898
#    , -1.4904, -1.4898, -1.4880, -1.4848, -1.4801, -1.4737, -1.4656, -1.4556, -1.4437
#    , -1.4302, -1.4152, -1.3992, -1.3829, -1.3669, -1.3521, -1.3393, -1.3295, -1.3233
#    , -1.3212, -1.3233, -1.3295, -1.3393, -1.3521, -1.3669, -1.3829, -1.3992, -1.4152
#    , -1.4302]]

#     move_robot(Q)


