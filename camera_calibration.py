import numpy as np
import cv2
import os
import glob
import matplotlib.pyplot as plt
import re

def camera_calibration(rows_chessboard=10, cols_chessboard=7, images=[]):

    plt.gray()
    nb_vertical = rows_chessboard
    nb_horizontal = cols_chessboard

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((nb_horizontal*nb_vertical,3), np.float32)
    objp[:,:2] = np.mgrid[0:nb_vertical,0:nb_horizontal].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.


    for img in images:
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (nb_vertical, nb_horizontal), None)

        # If found, add object points, image points (after refining them)
        if ret:

            # Refine the corner locations for better accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners\n",
            # img = cv2.drawChessboardCorners(img, (nb_vertical,nb_horizontal), corners,ret)
            # cv2.imshow('img',img)
            # cv2.waitKey(10000)

        #cv2.destroyAllWindows()


    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    h,  w = images[0].shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    # is this matrix in meters??
    rvecs = np.array(rvecs)
    tvecs = np.array(tvecs)
    rotation_matrix, _ = cv2.Rodrigues(rvecs[0]) 
    print(f"This is rotation_matrix: {rotation_matrix}")
    print(f"This is tvecs: {tvecs[1]}")
    return [newcameramtx, mtx, dist, roi]


def undistort(frame):

    # Plug in camera calibration parameters

    # New camera matrix for undistorted images (3x3)

    newcammtx = np.array([[493.59710031,   0,         425.22959262],
                      [0,         612.7970329,  245.57518965],
                      [0,           0,           1        ]])
    
    # Camera matrix for distorted images (3x3)
    
    mtx = np.array([[700.83379752,   0,         346.08857648],
                [  0,         701.04874854, 243.2621646 ],
                [  0,           0,           1        ]])
    
    # Distortion parameters (1x5)
    
    dist = np.array([-7.50841412e-02,  1.92152719e+00, -3.29816910e-03,  3.44972385e-03, -8.63153203e+00]).reshape(-1, 1)
    
    # Region of interest (x, y, w, h)
    
    x, y, w, h = np.array([181, 35, 449, 417])

    und = cv2.undistort(frame, mtx, dist, None, newcammtx)
    
    und = und[y:y+h, x:x+w]
    
    return und

# output_dir = "calibration_images"
# image_files = [os.path.join(output_dir, f) for f in os.listdir(output_dir) if f.endswith('.jpg')]
# images = [cv2.imread(img_file) for img_file in image_files]
# newcammtx, mtx, dist, roi = camera_calibration(images=images)
