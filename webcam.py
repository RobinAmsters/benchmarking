#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 15 09:55:40 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Aruco marker video analysis functions
"""

import cv2
import pickle
import time

import cv2.aruco as aruco
import numpy as np

#==============================================================================
#               WEBCAM VIDEO FILE FUNCTIONS
#==============================================================================
def count_frames_manual(video_file):
    """
        Funtion that counts the total number of frames in a video file.
        This is a very unoptimized implementation
    """
    
	# initialize the total number of frames read
    total = 0
    video = cv2.VideoCapture(video_file)

	# loop over the frames of the video
    while True:
		# grab the current frame
        (grabbed, frame) = video.read()

		# check to see if we have reached the end of the
		# video
        if not grabbed:
            break

		# increment the total number of frames read
        total += 1

	# return the total number of frames in the video file
    return total

def get_webcam_reference(video_file, cam_params_file, dictionary, marker_size, board, show_video=False, save_output=False, output_file_name='output.avi'):
    """
        Function that returns the position and orientation of a marker from its
        initial position.

        INPUTS:
            - video_file: path to a video file to be processed
            - cam_params_file: pickle file containing parameters from camera calibration
            - dictionary: aruco predifined dictionary used to generate markers
            - board: aruco marker board
            - marker_size: size of marker to detect in meters
            - show_video (default=False): play video with detection results. Video playback can be stopped by pressing q.
            - save_output (default=False): save the detection output to a video file
            - output_file_name (default='output.avi'): name of output video file

        OUTPUTS:
            - all_tvec: marker coordinates of each frame [x, y, z]
            - all_rvec: marker orientations of each frame [x, y, z]

    """

    # Open video file and get number of frames
    print('Preprocessing: counting number of frames')
    n_frames = count_frames_manual(video_file)
    cap = cv2.VideoCapture(video_file)

    # Parameters from camera calibration
    cal = pickle.load(open(cam_params_file, "rb" ))
    cMat = cal[0]
    dist = cal[1][0]

    # Initialize collections
    all_tvec = list()
    all_rvec = list()
    print('Tracking marker')

    if save_output:
        # Define the codec and create VideoWriter object
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        out = cv2.VideoWriter(output_file_name,fourcc, 29.0, size, True)  # 'False' for 1-ch instead of 3-ch for color

    parameters =  aruco.DetectorParameters_create() # Obtain detection parameters

    # Capture frame-by-frame
    for i in range(n_frames):

        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert to grayscale
        gray = frame

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        corners, ids, rejectedImgPoints, recoveredIdxs = aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedCorners=rejectedImgPoints, cameraMatrix=cMat, distCoeffs=dist)

        if ids is not None:

            # Obtain rotation and translation vectors
            rObject, tObject, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=cMat, distCoeffs=dist) # corners, size of markers in meters, [3x3] intrinsic camera parameters, 5 distortion coefficients
            rvec = [rObject.item(0), rObject.item(1), rObject.item(2)]
            tvec = [tObject.item(0), tObject.item(1), tObject.item(2)]

            all_rvec.append(rvec)
            all_tvec.append(tvec)

            # show information on image
            frameWithMarkers = aruco.drawDetectedMarkers(gray, corners, ids=ids) # Draw marker borders
            cv2.putText(frameWithMarkers, "ID: " + str(ids), (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)  # Show detected IDs
            aruco.drawAxis(frameWithMarkers, cMat, dist, np.array(rvec), np.array(tvec), 0.1) #Draw Axis

            # Display the resulting frame
            if show_video:
                cv2.imshow('frame',frameWithMarkers)

        else:
            # Display: no IDs
            cv2.putText(gray, "No IDs", (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv2.LINE_AA)
            if show_video:
                cv2.imshow('frame',gray)

        if save_output:
            out.write(gray)

        # Stop when q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    if save_output:
        out.release()
    cv2.destroyAllWindows()

    return np.array(all_tvec), np.array(all_rvec)
