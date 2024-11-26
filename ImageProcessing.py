import cv2
import numpy as np
from matplotlib import pyplot as plt



def get_circles(bgr_image,min_radius = 0, max_radius = 50, graphs = 0 ):
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 20, 200)
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=200, param2=20, minRadius=min_radius, maxRadius=max_radius)

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
        if np.mod(int(color[0]) - int(desired_color_hsv[0]),255) <= 7:
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

def get_red_centers_main():
    centers = []
    return centers