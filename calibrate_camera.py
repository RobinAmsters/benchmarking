#!/usr/bin/env python2
# -*- coding: utf-8 -*

""" 
Additional info:
    https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html

"""
import numpy as np
import cv2
import glob
import pickle

from file_select_gui import get_directory_path

# Save calibration data
save_params = True

# checkerboard Dimensions
cbrow = 6 # Checkerboard rows
cbcol = 7  # Checkerboard columns
square_size = 107.6 /1000 # Size of checkerboard square in millimeters, set to 1000 if square size is unknown
# image_folder = get_directory_path('Select folder containing calibration images ')
image_folder = '/home/quinten/Documents/vakantiejob/results/Video/images'
# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow * cbcol, 3), np.float32)
objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)*square_size

# Arrays to store object ponts and image points from all the images.
objpoints = [] # 3D points in real world space
imgpoints =  [] # 2D points in image plane

images = glob.glob(image_folder+'/*.jpg')
if images is None or not images:
    images = glob.glob(image_folder+'/*.png')

for fname in images:

    # Read image convert to grayscale
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 131 ,10)

    cv2.imshow('gray', gray)
    cv2.imshow('image', img)
    cv2.imshow('binary', binary)

    cv2.waitKey(0)

    print('Processing image: ' + fname)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(binary, (cbcol, cbrow), None)
    
    # If found, add object points, image points (afte refining them)
    if ret == True:
        print('Chess board pattern found')
        objpoints.append(objp)
			
        corners2 = cv2.cornerSubPix(binary, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
		
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (cbcol,cbrow), corners2, ret)
        # cv2.imwrite(image_folder +'/detected_corners/" + fname + "_chess.jpg", img) # Save image
			
        # Make window and show image
        frame_name = 'detected chessboard'
        cv2.namedWindow(frame_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(frame_name, 1920,1080)
        cv2.imshow(frame_name, img)
        cv2.waitKey(250)

retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None, criteria=criteria)
cv2.destroyAllWindows()

mean_error = 0 # Reprojection error, should be as close to zero as possible
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) ) 

if save_params:
    cal = [cameraMatrix, distCoeffs]
    f = open(('calibration_data.pckl'), 'wb')
    pickle.dump(cal, f)
    f.close()
