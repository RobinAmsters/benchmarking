#!/usr/bin/env python2
# -*- coding: utf-8 -*

""" 
Additional info:
    https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html
    
TODO: Re-projection Error
"""
import numpy as np
import cv2
import glob

from file_select_gui import get_directory_path

# checkerboard Dimensions
cbrow = 6
cbcol = 8
image_folder = get_directory_path('Select folder containing calibration images')

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow * cbcol, 3), np.float32)
objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)

# Arrays to store object ponts and image points from all the images.
objpoints = [] # 3D points in real world space
imgpoints =  [] # 2D points in image plane

images = glob.glob(image_folder+'/*.jpg')

for fname in images:

    # Read image convert to grayscale
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    print('Processing image: ' + fname)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (cbcol, cbrow), None)
    
    # If found, add object points, image points (afte refining them)
    if ret == True:
        print('Chess board pattern found')
        objpoints.append(objp)
			
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
		
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (cbcol,cbrow), corners2, ret)
        # cv2.imwrite("/chessboard_images/detected_corners/" + fname + "_chess.jpg", img) # Save image
			
        # Make window and show image
        frame_name = 'detected chessboard'
        cv2.namedWindow(frame_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(frame_name, 1920,1080)
        cv2.imshow(frame_name, img)
        cv2.waitKey(250)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None, criteria=criteria)

cv2.destroyAllWindows()
