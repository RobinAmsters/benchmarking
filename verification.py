#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 14:10:11 2017

@author: Robin Amsters

File to postprocess data from experiments. Position estimations of the robot
(from .bag files) are compared to position estimations from the krypton K600
coordinate measuring machine (.mat files)

TODO:
    - Test robot estimate function
    - Add function for light intensities
    - Check coordinate frames
"""
import cv2
import math
import operator
import pickle
import rosbag
import scipy.io
import sys
import time

#import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
import VLP_classes as vlp

from FileSelectGui import getFilePath
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
plt.rc('text', usetex=False)
#import matplotlib
#matplotlib.interactive(True)
#==============================================================================
#               GENERAL/OTHER FUNCTIONS
#==============================================================================
def rotate_point(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    
    return qx, qy

#==============================================================================
#               BAG FILE FUNCTIONS
#==============================================================================

def get_intensities(filePath):
    """
        Function to obtain intensity measurements contained in bag files.Is is 
        assumed that the measurements are published on the /light_intensity 
        topic.
    """
    
    # Open bag file
    bag = rosbag.Bag(filePath)
    
    # Intialize variables
    all_intensity = np.array([])
    all_t = np.array([])
    first = 1
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/light_intensity']):
        
        if first:
                t_start = msg.header.stamp
                first = 0
                
        all_intensity = np.append(all_intensity, msg.intensity)
        
        t = msg.header.stamp - t_start
        t = t.to_sec()
        all_t = np.append(all_t, t)
        
    # Close bag file
    bag.close()
    
    return all_intensity, all_t

def get_robot_estimate(filePath):
    """
        Function to obtain robot position estimates contained in bag files.
        It is assumed that these position estimates are published on the /pose
        topic.
    """
    
    # Open bag file
    bag = rosbag.Bag(filePath)
    
    #      Intialize variables
    all_x = np.array([])
    all_y = np.array([])
    all_t = np.array([])
    first = 1
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/pose']):
        
        if first:
                t_start = msg.header.stamp
                first = 0
        
        t = msg.header.stamp - t_start
        t = t.to_sec()
        all_t = np.append(all_t, t)
        all_x = np.append(all_x, msg.pose.x)
        all_y = np.append(all_y, msg.pose.y)
            
    # Remap to world coordinate frame
    x = -all_y
    y = all_x
    z = np.zeros(len(x))
        
    # Close bag file
    bag.close()
    
    return x, y, z, all_t

def get_verification_data(filePath):
    """
        Function to obtain verification data contained in bag files. It is 
        assumed that these bag files are an output of play_bag.launch
    """
    # Open bag file
    bag = rosbag.Bag(filePath)
    
    #      Intialize variables
    h = np.array([])
    x_est = np.array([])
    y_est = np.array([])
    theta_est = np.array([])
    
    z = np.array([])
    x_ref = np.array([])
    y_ref = np.array([])
    theta_ref = np.array([])
    
    all_t = np.array([])
    first = 1
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/verification_data']):
        
        if first:
                t_start = msg.stamp
                first = 0
        
        t = msg.stamp - t_start
        t = t.to_sec()
        all_t = np.append(all_t, t)
        
        # Get robot position estimate
        x_est = np.append(x_est, msg.pose_est.x)
        y_est = np.append(y_est, msg.pose_est.y)
        theta_est = np.append(theta_est, msg.pose_est.theta)
            
        # Get reference position
        x_ref = np.append(x_ref, msg.pose_ref.x)
        y_ref = np.append(y_ref, msg.pose_ref.y)
        theta_ref = np.append(theta_ref, msg.pose_ref.theta)
        
        # Get measurement
        z = np.append(z, msg.z)
        
        # Get model
        h = np.append(h, msg.h)    
        
    pose_est = [x_est, y_est, theta_est]    
    pose_ref = [x_ref, y_ref, theta_ref]           
        
    # Close bag file
    bag.close()
    
    return pose_est, pose_ref, z, h, all_t

#==============================================================================
#               VIVE FUNCTIONS
#==============================================================================
def get_VIVE_reference(VIVE_data_path):
    # Defining local variables
    fileHandle = open(VIVE_data_path, 'r') #Internal name for file
    lines = fileHandle.readlines() #All lines in the file
    
    all_t = []
    all_x = []
    all_y = []
    all_z = []
    all_qx = []
    all_qy = []
    all_qz = []
    all_qw = []
    dr_ref = []

    for line in lines:
     
        if line.count(' ') :
            # If the line contains a space and the measurement has started, add this line to the currentMeasurement matrix
            measurement = [float(i) for i in line.split()]
            all_t = np.append(all_t, measurement[0])
            all_x = np.append(all_x, measurement[1])
            all_y = np.append(all_y, measurement[2])
            all_z = np.append(all_z, measurement[3])
            all_qx = np.append(all_qx, measurement[4])
            all_qy = np.append(all_qy, measurement[5])
            all_qz = np.append(all_qz, measurement[6])
            all_qw = np.append(all_qw, measurement[7])
    
    return all_x, all_y, all_z
    

#==============================================================================
#               WEBCAM VIDEO FILE FUNCTIONS
#==============================================================================
def count_frames_manual(videoFilePath):
    """
        Funtion that counts the total number of frames in a video file.
        This is a very unoptimized implementation
    """
    
	# initialize the total number of frames read
    total = 0
    video = cv2.VideoCapture(videoFilePath)
 
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

def get_webcam_reference(videoFilePath, cParamsFilePath, dictionary, markerSize, pose_timeout=5, show_video=False):
    """
        Function that returns the position and orientation of a marker from its
        initial position. The input is a video file containing marker
    """
    # Open video file and get number of frames
    print('Preprocessing: counting number of frames')
    n_frames = count_frames_manual(videoFilePath)
    cap = cv2.VideoCapture(videoFilePath)
    
    # Parameters from camera calibration
    cal = pickle.load(open("tst_chessboard_lab.p", "rb" ))
#    cal = pickle.load(open(cParamsFilePath, "rb" ))
    cMat = cal[0]
    dist = cal[1]
    
    # Initialze parameters to determine initial position
    pose_0_set = False
    pose_0 = ()
    t_start = time.time()
    t_set_pose_0 = t_start + pose_timeout
    
    # Initialize collections
    all_tvec = np.array([[],[],[]])
    all_rvec = np.array([[],[],[]])
    print('Postprocessing: tracking marker')
    # Define the codec and create VideoWriter object
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter('output.avi',fourcc, 29.0, size, False)  # 'False' for 1-ch instead of 3-ch for color
    parameters =  aruco.DetectorParameters_create() # Obtain detection parameters
#    parameters.minCornerDistanceRate = 0.1
#    parameters.minOtsuStdDev = 0.1
#    parameters.maxErroneousBitsInBorderRate = 0.5
#    parameters.adaptiveThreshConstant = 30
#    parameters.perspectiveRemovePixelPerCell = 1000
#    parameters.polygonalApproxAccuracyRate = 0.1
    # Capture frame-by-frame
    for i in range(n_frames):
        
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert to grayscale
        gray = frame
#        print(help(parameters))
        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=parameters)   
        corners, ids, rejectedImgPoints, recoveredIdxs = aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedCorners=rejectedImgPoints, cameraMatrix=cMat, distCoeffs=dist)
        
        if ids is not None:  
            
            # Obtain rotation and translation vectors
            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerSize, cameraMatrix=cMat, distCoeffs=dist) # corners, size of markers in meters, [3x3] intrinsic camera parameters, 5 distortion coefficients 
            rvec = np.array([rvec.item(0), rvec.item(1), rvec.item(2)])
            tvec = np.array([tvec.item(0), tvec.item(1), tvec.item(2)])
    
            if time.time() >= t_set_pose_0 and not pose_0_set:
                pose_0 = (rvec, tvec)
                pose_0_set = True
            
            if pose_0_set:
                
                rvec_0 = pose_0[0]
                tvec_0 = pose_0[1]
                
                tvec_n = tvec - tvec_0
                rvec_n = rvec - rvec_0
                
                t0 = np.append(all_tvec[0], tvec[0])
                t1 = np.append(all_tvec[1], tvec[1])
                t2 = np.append(all_tvec[2], tvec[2])
                all_tvec = np.array([t0,t1,t2])
                
                r0 = np.append(all_rvec[0], rvec[0])
                r1 = np.append(all_rvec[1], rvec[1])
                r2 = np.append(all_rvec[2], rvec[2])
                all_rvec = np.array([r0,r1,r2])
                
                r = np.sqrt(np.power(tvec_n[0],2) + np.power(tvec_n[1],2))
                           
#                print('Translation from initial position: ', tvec_n)
#                print('Rotation from initial position: ', rvec_n)
#                print('Total distance from initial position: ', r)
             
            # show information on image
            frameWithMarkers = aruco.drawDetectedMarkers(gray, corners) # Draw marker borders
            cv2.putText(frameWithMarkers, "ID: " + str(ids), (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)  # Show detected IDs
            aruco.drawAxis(frameWithMarkers, cMat, dist, rvec, tvec, 0.1) #Draw Axis  
            
            # Display the resulting frame
            if show_video:
                cv2.imshow('frame',frameWithMarkers)
		out.write(gray)
            
        else:  
            # Display: no IDs
            cv2.putText(gray, "No IDs", (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv2.LINE_AA)  
            if show_video:
                cv2.imshow('frame',gray)
		out.write(gray)


        # Stop when q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
 
    # When everything done, release the capture
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    return all_tvec, all_rvec

#==============================================================================
#               KRYPTON MAT FILE FUNCTIONS
#==============================================================================


def get_krypton_reference(filePath):
    """
        Function that returns the x,y and z positions contained in a .mat file
        containing krypton camera measurements of a dynamic frame.
    """
    # Open .mat file and get raw data
    mat = scipy.io.loadmat(filePath)
    data = mat.get('dynamicrobotframe')
    x_filtered, y_filtered, z_filtered, x_first, y_first, z_first = filter_krypton_data(data)
    t_real = np.linspace(0.0, len(z_filtered)*0.02, len(z_filtered)) # Assume sampling at 50Hz
    
    # Take initial position as 0 and convert from mm to m
    x_filtered = (x_filtered - x_first)/1000.0
    y_filtered = (y_filtered - y_first)/1000.0
    z_filtered = (z_filtered - z_first)/1000.0
    
    # Transform to world coordinates
    x_real = x_filtered
    y_real = y_filtered
    z_real = z_filtered
    
    return x_real, y_real, z_real, t_real

def filter_krypton_data(data):
    """
        Function to filter camera data. The camera either gives zero or a fixed
        large number when it loses vision. This function replaces this with NaN.
        
    """
    
    x_filtered = data[0]
    y_filtered = data[1]
    z_filtered = data[2]
    
    # Points to filter out, this is the coordinate that the camera resets to when it loses vision
    filter_x = 4894
    filter_y = 268
    filter_z = 682
    filter_lim = 0.01 # Percentage range around the filter point
    
    # Filter points where camera lost vision
    for i in range(len(x_filtered)):
        x = x_filtered[i]
        y = y_filtered[i]
        z = z_filtered[i]
        
        if x >= filter_x*(1-filter_lim) and x<= filter_x*(1+filter_lim):
            x_filtered[i] = np.NaN
            
        if y >= filter_y*(1-filter_lim) and y<= filter_y*(1+filter_lim):
            y_filtered[i] = np.NaN
            
        if z >= filter_z*(1-filter_lim) and z<= filter_z*(1+filter_lim):
            z_filtered[i] = np.NaN
            
    # Get first point where camera has vision
    x_first = x_filtered[0]
    x_first_set = False
    y_first = y_filtered[0]
    y_first_set = False
    z_first = z_filtered[0]
    z_first_set = False    
    
    for i in range(len(x_filtered)):
        x = x_filtered[i]
        y = y_filtered[i]
        z = z_filtered[i]
        
        if not np.isnan(x) and not x_first_set:
            x_first = x
            x_first_set = True
            
        if not np.isnan(y) and not y_first_set:
            y_first = y
            y_first_set = True
            
        if not np.isnan(z)and not z_first_set:
            z_first = z
            z_first_set = True
            
    return x_filtered, y_filtered, z_filtered, x_first, y_first, z_first

#==============================================================================
#               OTHER FUNCTIONS       
#==============================================================================
def simulate_order(room, x, y, m):
    """
        Simulate the output of a receiver at specific coordinates for 
        a different lambertian order
    """
    room.Tx1.m = m
    room.Tx2.m = m
    room.Tx3.m = m
    room.Tx4.m = m
    
    h = room.predict_measurement_xy(x, y)[2] 
    scale_factor = z[0]/h[0]
    
    h = scale_factor*h
    
    return h, scale_factor

def get_dr(all_x_est, all_y_est, all_x_ref, all_y_ref):
    
    dr = np.ones(len(all_x_est))
    
    for i in range(len(all_x_est)):
        x_est = all_x_est[i]
        y_est = all_y_est[i]
        x_ref = all_x_ref[i]
        y_ref = all_y_ref[i]
        
        dr[i] = math.sqrt((x_ref-x_est)**2 + (y_ref-y_est)**2)
        
    return dr
        
#==============================================================================
#               GET AND PLOT DATA
#==============================================================================

if __name__ == "__main__":

    if len(sys.argv) == 2:
        save = bool(sys.argv[1])
        I_Tx = 1.05               # Driving current [A]
    elif len(sys.argv)  == 3:
        save = bool(sys.argv[1])
        I_Tx = float(sys.argv[2])
    else:    
        save = False    # Save the resulting plots
        I_Tx = 1.05               # Driving current [A]
    plot_interval = 2
    dpi=300
    figNum = 1
    titleSize=14
    legendSize=12
    axesSize=12
    labelSize=12
    dpi=1000
    legendLoc = 3
    print(save)
#%%                   GET DATA FROM FILES

    #   Select files with GUI
    bagFilePath = getFilePath("Select .bag file").name
    
    #   Get path ID from name
    if bagFilePath.count('path_1'):
        path_ID = 1
    elif bagFilePath.count('path_2'):
        path_ID = 2
    elif bagFilePath.count('mapping'):
        path_ID = 3
    else:
        path_ID = 5
    
    #   Get robot position estimate    
    pose_est, pose_ref, z, h, all_t = get_verification_data(bagFilePath)
    x_est = pose_est[0]
    y_est = pose_est[1]
    
    #   Translate to world coordinate frame  
    if path_ID == 1:
        x_rot = []
        y_rot = []
        for i in range(len(pose_ref[0])):
            point = [pose_ref[0][i], pose_ref[1][i]]
            x_r, y_r = rotate_point([0,0], point, 3.14)
            x_rot.append(x_r+2.3)
            y_rot.append(y_r)
        x_ref = x_rot
        y_ref = y_rot
    elif path_ID == 2:
        x_rot = []
        y_rot = []
        for i in range(len(pose_ref[0])):
            point = [pose_ref[0][i], pose_ref[1][i]]
            x_r, y_r = rotate_point([0,0], point, 1.54)
            x_rot.append(x_r+2.3)
            y_rot.append(y_r-2.1875 )
        x_ref = x_rot
        y_ref = y_rot
    elif path_ID ==3:
        x_rot = []
        y_rot = []
        for i in range(len(pose_ref[0])):
            point = [pose_ref[0][i], pose_ref[1][i]]
            x_r, y_r = rotate_point([0,0], point, 1.57)
            x_rot.append(x_r+2.3)
            y_rot.append(y_r-2.1875 )
        x_ref = x_rot
        y_ref = y_rot
    else:
        x_ref = pose_ref[0]
        y_ref = pose_ref[1]
    
#%%                   VERIFIFY MODEL
    
    # Light intensity model at reference locations
#    m_orig = 1.1408403412173747 # Lambertion order
#    scale_factor = 1.0574
    m_orig = 1.0
    phi_v_nom = 4275       # Nominal luminous flux [lm]
    I_Tx2 = 0.01
    pos_Tx1 = np.array([0.0,0.0])       # Position of LED1 [x,y]
    pos_Tx2 = np.array([10000,10000])   # Position of LED2 [x,y]
    pos_Tx3 = np.array([10000,10000])   # Position of LED2 [x,y]
    pos_Tx4 = np.array([10000,10000])   # Position of LED2 [x,y]
#    pos_Tx2 = np.array([2.3,-2.1875])    # Position of LED2 [x,y]
#    pos_Tx3 = np.array([2.3,2.1875])     # Position of LED3 [x,y]
#    pos_Tx4 = np.array([-2.3,2.1875])    # Position of LED4 [x,y]
    
    # Receiver parameters
    A_Rx = 7.02*10**(-6)    # Area of receiver [m^2]
    R_max = 0.62            # Peak responsivity of PD [A/W]
    G = 2.96*10**6              # Amplifier gain [V/A]
    pos_Rx = np.array([0,0])          # Initial position of receiver [x,y]
    
    # Room parameters
    dz = 2.8
    I_bg = 190*10**-6 # Background current noise [A]
    
    #                           Simulation objects
    Tx1 = vlp.LED(pos_Tx1, m_orig, I_Tx, phi_v_nom)
    Tx2 = vlp.LED(pos_Tx2, m_orig, I_Tx2, phi_v_nom)
    Tx3 = vlp.LED(pos_Tx3, m_orig, I_Tx2, phi_v_nom)
    Tx4 = vlp.LED(pos_Tx4, m_orig, I_Tx2, phi_v_nom)
    
    Rx = vlp.Receiver(pos_Rx, A_Rx, R_max, G)
    room = vlp.Room(Tx1, Tx2, Tx3, Tx4, Rx, dz=dz)
    
    # Get models
    h_pos_ref_orig = room.predict_measurement_xy(x_ref, y_ref)[2] 
    
    all_m = [0.5, 1.0]
    h_pos_ref_scaled, scale_factor = simulate_order(room, x_ref, y_ref, m_orig)
#    h_pos_ref_0 = simulate_order(room, x_ref, y_ref, all_m[0])
#    h_pos_ref_1 = simulate_order(room, x_ref, y_ref, all_m[1])
#    h_pos_ref_2 = simulate_order(room, x_ref, y_ref, all_m[2])
#    h_pos_ref_3 = simulate_order(room, x_ref, y_ref, all_m[3])
#    h_pos_ref_4 = simulate_order(room, x_ref, y_ref, all_m[4])
#    h_pos_ref_5 = simulate_order(room, x_ref, y_ref, all_m[5])
    
    # Calculate error
    dV = h_pos_ref_orig - z
    dV_scaled = h_pos_ref_scaled - z
    
    max_index, max_value = max(enumerate(z), key=operator.itemgetter(1))
    
    # Print primary parameters
    print('m = ' + str(m_orig))
#    print('scale factor = ' + str(scale_factor))
    print('scale factor = ', scale_factor)
    print('I_Tx1 = ' + str(I_Tx))
    print('max_meas = ' + str(max_value) + ' V at ['+ str(x_ref[max_index]) + ', ' + str(y_ref[max_index]) + ']')
    
    
#%%                   CALCULATE POSITIONING ERROR
    dr = get_dr(x_est, y_est, x_ref, y_ref)
       
#%%                   PLOTTING
#==============================================================================             
    fig_name = '_model_error'
    fig = plt.figure(figNum)
    fig.clf()
    fig.canvas.set_window_title(fig_name)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
        
    # Plot raw intensities  
    ax = fig.add_subplot(211)
    ax.plot(all_t, z, label='Measurement')
#    ax.plot(all_t, h_pos_ref_orig, label=('Model'))
    ax.plot(all_t, h_pos_ref_scaled, label=('Scaled model'))
#    ax.plot(all_t, h_pos_ref_0, label=('Scaled model at reference location, m=' + str(all_m[0])))
#    ax.plot(all_t, h_pos_ref_1, label=('Scaled model at reference location, m=' + str(all_m[1])))
#    ax.plot(all_t, h_pos_ref_2, label=('Model at reference location, m=' + str(all_m[2])))
#    ax.plot(all_t, h_pos_ref_3, label=('Model at reference location, m=' + str(all_m[3])))
#    ax.plot(all_t, h_pos_ref_4, label=('Model at reference location, m=' + str(all_m[4])))
#    ax.plot(all_t, h_pos_ref_5, label=('Model at reference location, m=' + str(all_m[5])))
    
    ax.set_xlim([0, max(all_t)])
    ax.set_ylim([0,max(max(z), max(h_pos_ref_orig))+0.2])
    ax.set_xlabel('Time [s]', fontsize=labelSize)
    ax.set_ylabel('Receiver output [V]', fontsize=labelSize)
    ax.set_title('Light intensity', fontsize=titleSize)
    plt.grid()
    plt.legend()
    
    # Plot model error  
    ax = fig.add_subplot(212)
    plt.scatter(x_ref, y_ref, c=dV_scaled, cmap='jet', vmin=-0.5, vmax=0.5, s=1, label='dV [V]')
    
    ax.set_xlabel('x-coordinate [m]', fontsize=labelSize)
    ax.set_ylabel('y-coordinate [m]', fontsize=labelSize)
    ax.set_title('Scaled model error [V]', fontsize=titleSize)
    plt.colorbar()
    plt.grid()
    
    if save:
        plt.savefig((bagFilePath[0:-4] + fig_name + '.png'), format='png', dpi=1000.0,bbox_inches='tight') 

    figNum +=1
#==============================================================================
#    fig_name = 'models'
#    fig = plt.figure(figNum)
#    fig.clf()
#    fig.canvas.set_window_title(fig_name)
#    
#    ax = fig.gca()
#    ax.plot(all_t, z, label='Measurement')
#    ax.plot(all_t, h_pos_ref, label='Model at reference location')
#    
#    all_m = np.linspace(0.0, 2.0, 11)
##    all_m = [0.55]
#    scale_point = 300
#    for i in range(len(all_m)):
#        m = all_m[i]
#        
#        room.Tx1.m = m
#        room.Tx2.m = m
#        room.Tx3.m = m
#        room.Tx4.m = m
#        #             GET MEASUREMENTS
#        receiver_output = room.predict_measurement_xy(x_ref, y_ref)[2]
##        scale_factor = z[scale_point]/receiver_output[scale_point]
##        print(scale_factor)
#        output_scaled = receiver_output*scale_factor
#        ax.plot(all_t, output_scaled, label=('Scaled model at reference location, m= ' + str(m)))
#
#    ax.set_ylim([0,max(max(z), max(h_pos_ref))+0.2])
#    ax.set_xlabel('Time [s]', fontsize=labelSize)
#    ax.set_ylabel('Receiver output [V]', fontsize=labelSize)
#    ax.set_title('Light intensity', fontsize=titleSize, y=1.05)
#    plt.legend()
#    
#    if save:
#        plt.savefig((bagFilePath[0:-4] + fig_name + '.eps'), format='eps', dpi=dpi,bbox_inches='tight') 
#        
#    figNum +=1
    
#==============================================================================
#    # Plot position estimates
#    fig_name = 'Robot position'
#    fig = plt.figure(figNum)
#    fig.clf()
#    fig.canvas.set_window_title(fig_name)
#
#    ax = fig.gca()
#    
#    ax.plot(x_ref, y_ref, label='laser scan matching position')
#    ax.plot(x_est, y_est, label='robot position estimate')
#    
#    ax.set_xlim([-3.1, 3.1])
#    ax.set_ylim([-3.1, 3.1])
#    ax.set_xlabel('X-coordinate [m]', fontsize=labelSize)
#    ax.set_ylabel('Y-coordinate [m]', fontsize=labelSize)
#    
#    plt.grid()
#    plt.legend(bbox_to_anchor=(0., -0.2, 1., .102), loc=legendLoc,
#           ncol=2, borderaxespad=0.0, fontsize=legendSize)
#    
#    if save:
#        plt.savefig((bagFilePath[0:-4] + fig_name + '.eps'), format='eps', dpi=dpi,bbox_inches='tight') 
#        
#    figNum +=1
        
#==============================================================================        
#    # Plot measurements with model        
#    fig_name = '_light_intensity'
#    fig = plt.figure(figNum)
#    fig.clf()
#    fig.canvas.set_window_title(fig_name)
#    figManager = plt.get_current_fig_manager()
#    figManager.window.showMaximized()
#
#     
#    ax = fig.add_subplot(211, projection='3d')
#    ax.plot_wireframe(x_ref, y_ref, z, cmap=cm.jet, label='Measurement at reference location')
##    ax.plot_wireframe(x_ref, y_ref, h_pos_ref_orig, color='orange', label='Model at reference location')
#    ax.scatter3D(pos_Tx1[0], pos_Tx1[1], dz, color='0.35', s=100, label='LED positions')
#    ax.scatter3D(pos_Tx2[0], pos_Tx2[1], dz, color='0.35', s=100)
#    ax.scatter3D(pos_Tx3[0], pos_Tx3[1], dz, color='0.35', s=100)
#    ax.scatter3D(pos_Tx4[0], pos_Tx4[1], dz, color='0.35',  s=100)
#    
#    ax.set_xlim([-3.1,3.1])
#    ax.set_ylim([-3.1,3.1])
#    ax.set_zlim([0,max(max(z), max(h_pos_ref_orig))+0.2])
#
#    plt.tick_params(axis='both', which='major', labelsize=labelSize)
#    plt.grid()
##    plt.legend(bbox_to_anchor=(0., -0.2, 1., .102), loc=legendLoc,
##           ncol=2, borderaxespad=0.0, fontsize=legendSize)
#    plt.legend()
#    
#    ax = fig.add_subplot(212, projection='3d')
#    ax.plot_wireframe(x_ref, y_ref, z, cmap=cm.jet, label='Measurement at reference location')
#    ax.plot_wireframe(x_ref, y_ref, h_pos_ref_scaled, color='green', label='Scaled model at reference location')
#    ax.scatter3D(pos_Tx1[0], pos_Tx1[1], dz, color='0.35', s=100, label='LED positions')
#    ax.scatter3D(pos_Tx2[0], pos_Tx2[1], dz, color='0.35', s=100)
#    ax.scatter3D(pos_Tx3[0], pos_Tx3[1], dz, color='0.35', s=100)
#    ax.scatter3D(pos_Tx4[0], pos_Tx4[1], dz, color='0.35',  s=100)
#    
#    ax.set_xlim([-3.1,3.1])
#    ax.set_ylim([-3.1,3.1])
#    ax.set_zlim([0,max(max(z), max(h_pos_ref_orig))+0.2])
#
#    plt.tick_params(axis='both', which='major', labelsize=labelSize)
#    plt.grid()
##    plt.legend(bbox_to_anchor=(0., -0.2, 1., .102), loc=legendLoc,
##           ncol=2, borderaxespad=0.0, fontsize=legendSize)
#    plt.legend()
#    
#    if save:
#        plt.savefig((bagFilePath[0:-4] + fig_name + '.eps'), format='eps', dpi=dpi,bbox_inches='tight')   
#    
#    figNum +=1            
#==============================================================================
    fig_name = '_positioning_error'
    fig = plt.figure(figNum)
    fig.clf()
    fig.canvas.set_window_title(fig_name)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()

    ax = fig.gca()
    ax.plot(all_t, dr)
    ax.set_xlabel('Time [s]', fontsize=labelSize)
    ax.set_ylabel('Positioning errror [m]', fontsize=labelSize)
    ax.set_xlim([0, max(all_t)])
    plt.grid()

    if save:
        plt.savefig((bagFilePath[0:-4] + fig_name + '.png'), format='png', dpi=dpi,bbox_inches='tight')   
    
    figNum +=1
   
    
    plt.show()
