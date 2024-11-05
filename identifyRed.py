import cv2
import numpy as np
from matplotlib import pyplot as plt
import imutils

def get_red_cordinates(bgr_image):
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([179, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    edges = cv2.Canny(mask, 20, 200)
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=200, param2=20, minRadius=0, maxRadius=50)
    centers = circles[0,:]

    return centers

def calibration(single_smartie):
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(single_smartie, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 20, 200)



bgr_img = cv2.imread("Screenshot 2024-11-05 at 11-26-07 smarties spread table - Pesquisa Google.png")
centers = get_red_cordinates(bgr_img)
output = bgr_img.copy()
for i in centers:
    cv2.circle(output,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),2)

b,g,r = cv2.split(output)       # get b,g,r
image = cv2.merge([r,g,b])
(h, w, d) = image.shape
print("width={}, height={}, depth={}".format(w, h, d))
plt.figure()
plt.imshow(image)
plt.show()
