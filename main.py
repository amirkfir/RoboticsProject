# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import cv2

from ImageProcessing import calibration, get_color_coardinates
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
        c.release()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
