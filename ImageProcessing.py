import cv2
import numpy as np
from matplotlib import pyplot as plt
from robot_control import *
from camera_calibration import *



def get_circles(bgr_image,min_radius = 0, max_radius = 50, graphs = 0 ):
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 20, 80)
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=80, param2=20, minRadius=min_radius, maxRadius=max_radius)

    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    mean_colors = []
    if circles is not None:
        for i in circles[0,:]:
            output = rgb_image.copy()
            mask = np.zeros_like(gray)
            cv2.circle(mask, (int(i[0]), int(i[1])), int(i[2]), 1, -1)
            output = cv2.bitwise_and(output, output, mask=mask)
            vec_output = output.reshape(-1, 3)
            vec_mask = mask.reshape(-1, 1).repeat(3, axis=1)
            avg_rgb = np.nanmean(np.where(vec_mask > 0, vec_output, np.nan), axis=0)
            avg_rgb = np.array([[avg_rgb]], dtype=np.uint8)
            mean_colors.append(np.squeeze(cv2.cvtColor(avg_rgb,cv2.COLOR_RGB2HSV)))

    if graphs:
        if circles is not None:
            output = hsv_image.copy()
            for i in range(len(circles[0,:])):
                cv2.circle(output, (int(circles[0,i,0]), int(circles[0,i,1])), int(circles[0,i,2]), mean_colors[i].tolist(), 5)
            plt.imshow(cv2.cvtColor(output, cv2.COLOR_HSV2RGB))
            plt.show()
    return circles, mean_colors


def get_color_coordinates(bgr_image,desired_color_hsv = 0,radius = 23, graphs = 0):
    circles, mean_colors = get_circles(bgr_image, round(radius-7),round(radius+7),graphs)
    red_circles = []
    idx = 0
    for color in mean_colors:
        if np.mod(int(color[0]) - int(desired_color_hsv[0]),255) <= 7 or np.mod(int(color[0]) - int(desired_color_hsv[0]),255)>=248:
            red_circles.append(idx)
        idx += 1

    if graphs:
        output = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        for i in red_circles:
            cv2.circle(output, (int(circles[0, i, 0]), int(circles[0, i, 1])), int(circles[0, i, 2]),
                       mean_colors[i].tolist(), 5)
        plt.imshow(cv2.cvtColor(output, cv2.COLOR_HSV2RGB))
        plt.show()

    if circles is not None:
        centers =circles[0,red_circles,:2]
        centers = centers[:, [1, 0]]
    else:
        centers = []
    return centers

# def calibration(single_smartie):
#     gray = cv2.cvtColor(single_smartie, cv2.COLOR_BGR2GRAY)
#     edges = cv2.Canny(gray, 20, 200)
#     circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20,
#                                param1=200, param2=20, minRadius=0, maxRadius=50)
#     output = single_smartie.copy()
#     output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
#     # gray = cv2.cvtColor(single_smartie, cv2.COLOR_BGR2GRAY)
#     # limits = [[200,200,200],[255,255,255]]
#     # thresh = cv2.inRange(single_smartie, limits[0], limits[1])
#     mask = np.zeros_like(gray)
#     if circles is not None:
#         for i in circles[0,:]:
#             cv2.circle(mask, (int(i[0]), int(i[1])), int(i[2]), 255, -1)
#     output = cv2.bitwise_and(output, output, mask=mask)
#     # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
#     # morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
#     # k = 255 - morph
#     # result = cv2.bitwise_and(single_smartie, single_smartie, mask=k)
#     vec_output = output.reshape(-1, 3)
#
#     color_mean = np.nanmean(np.where(vec_output > 0, vec_output, np.nan), axis=0)
#
#     if circles is not None:
#         for i in circles[0,:]:
#             cv2.circle(output, (int(i[0]), int(i[1])), int(i[2]), color_mean, -1)
#     # hsv_image = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
#     plt.imshow(cv2.cvtColor(output, cv2.COLOR_HSV2RGB))
#     plt.show()
#     return color_mean, circles[0,0,2]




# bgr_img = cv2.imread("Screenshot 2024-11-05 at 11-26-07 smarties spread table - Pesquisa Google.png")
# centers = get_red_cordinates(bgr_img)
# output = bgr_img.copy()
# for i in centers:
#     cv2.circle(output,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),2)
#
# b,g,r = cv2.split(output)       # get b,g,r
# image = cv2.merge([r,g,b])
# (h, w, d) = image.shape
# print("width={}, height={}, depth={}".format(w, h, d))
# plt.figure()
# plt.imshow(image)
# plt.show()

def calibration(single_smartie,graphs = 0):
    circles, mean_colors = get_circles(single_smartie,graphs=graphs)
    return mean_colors[0], circles[0, 0, 2]

def get_red_centers_main(portHandler, packetHandler,current_position):
    # Put the robot in the desired position for calibration
    q1, q2, q3, q4 = current_position
    # portHandler, packetHandler = initial_pos_set(initial_pos=[q1, q2, q3, q4])
    # move_robot_to_point([0, -30, -65, -83], portHandler, packetHandler)
    c = None
    try:
        # Set port number for robot's camera
        port_number = 2
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
            # frame = undistort(frame)
            color, radius = calibration(frame, 1)

        input("Arrange multiple smarties layout and press enter")
        
        count = 0
        for _ in range(20):
            flag, frame = c.read()

        if flag:
            # frame = undistort(frame)
            centers = get_color_coordinates(frame, color, radius, 1)
            for center in centers:
                count += 1
                print(center)
        
        c.release()

        input("Calibration success? Press enter")

        # Calculate the forward kinematics to get the camera's position wrt world frame

        _, T50 = forward_kinematics(q1, q2, q3, q4)
        T50 = np.array(T50)
        print(T50)

        # Get the goal positions (x, y, z) coordinates wrt world frame for red smarties
        # We know the plane of the table where the smarties sit

        z = -22  # Table plane
        cam_pos = T50[:3, 3]  # camera position

        print(f'This is z: {z} \nAnd this is z_c: {cam_pos[2]}')
        goal_positions = []

        if count == 1:
            goal_position = goal_pos_finder(centers, cam_pos, z)
            goal_positions.append(goal_position+ np.array([120,-5,0]))
        else:
            for center in centers:
                goal_position = goal_pos_finder(center, cam_pos, z)
                goal_positions.append(goal_position+np.array([120,-5,0]))

        for i, pos in enumerate(goal_positions):
            print(f'This is position number {i}: {pos}')


        return goal_positions