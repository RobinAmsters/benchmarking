#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 31 11:49:26 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be



"""

import bag_data as bag
import numpy as np

from file_select_gui import get_file_path

def get_vector_angle(p1, p2):
    """!
        @brief Calculate the angle between two vectors in radians ranging from zero to 2pi

        @param p1: first vector, should be an arrays of the form [x,y]
        @param p1: first vector, should be an arrays of the form [x,y]

        @return theta: the angle between p1 and p2
    """
    ang1 = np.arctan2(p1[0], p1[1])
    ang2 = np.arctan2(p2[0], p2[1])
    theta = (ang1 - ang2) % (2 * np.pi)

    return theta

def align_time_index(time_1, time_2, slack=0.01):
    """!
        @brief return indexes of two time series for which their timestamps are approximately equal.
        @details Very unoptimized approach, could possibly improved by resampling:
            https://jakevdp.github.io/PythonDataScienceHandbook/03.11-working-with-time-series.html
            https://machinelearningmastery.com/resample-interpolate-time-series-data-python/

        @param time_1: list containing the first time series (entries should be numbers)
        @param time_2: list containing the second time series (entries should be numbers)
        @param slack: allowed mismatch between timestamps of the series (in seconds)

        @return index_1: indices of the first time series for which the corresponding entries are an approximate match
        for the entries of index_2 in time_2
        @return index_2: indices of the second time series for which the corresponding entries are an approximate match
        for the entries of index_1 in time_1

    """
    index_1 = []
    index_2 = []

    # Loop over both time series and find best match for each timestamp
    for i in range(len(time_1)):

        time_stamp_1 = time_1[i]
        dt_min = 1000.0  # Initialize as large number, to be overridden first

        # Find best match by checking every entry in the second time series, hence the very unoptimized warning
        for j in range(len(time_2)):

            time_stamp_2 = time_2[j]
            dt = abs(time_stamp_1 - time_stamp_2)

            if (dt <= slack) and (dt < dt_min) and (j not in index_2):
                # Make sure the time difference is less than specified and at a minimum

                index_1.append(i)
                index_2.append(j)
                dt_min = dt

    return index_1, index_2

def get_robot_pose(hedge_1_pos, hedge_1_time, hedge_2_pos, hedge_2_time, slack=0.1):
    """!
    @brief returns the ground truth robot pose based on marvelmind hedgehog locations

    @param hedge_1_pos: Position of first tracked hedgehog [[x_n, y_n, z_n]]
    @param hedge_1_time: Timestamps of first hedgehog position measurements [ms since start]
    @param hedge_2_pos: Position of second tracked hedgehog [[x_n, y_n, z_n]]
    @param hedge_2_time: Timestamps of second hedgehog position measurements [ms since start]
    @return robot_pose: Robot pose [[x_n, y_n, theta_n]], for now taken as average of two hedgehog coordinates
    """

    # Align timestamps of hedgehog locations so that the robot center location and robot heading can be obtained from
    # their know translation relative to the robot center
    hedge_1_index, hedge_2_index = align_time_index(hedge_1_time, hedge_2_time, slack=slack)

    robot_pose = np.empty([len(hedge_1_index), 3])  # robot pose: [x_center, y_center, theta]
    robot_time = np.empty([len(hedge_1_index)])  # robot pose: [x_center, y_center, theta]

    for i in range(len(hedge_1_index)):
        hedge_1_i = hedge_1_index[i]
        hedge_2_i = hedge_2_index[i]

        # Get robot position from hedgehog coordinates, assume hedgehogs ar
        robot_pose[i, 0] = np.average([hedge_1_pos[hedge_1_i, 0], hedge_2_pos[hedge_2_i, 0]])
        robot_pose[i, 1] = np.average([hedge_1_pos[hedge_1_i, 1], hedge_2_pos[hedge_2_i, 1]])

        # Get angle between hedgehogs, to be used as ground truth for robot heading
        robot_pose[i, 2] = get_vector_angle(hedge_1_pos[hedge_1_index[i]], hedge_2_pos[hedge_2_index[i]])

        # Take timestamps of robot pose the same as aligned timestamps of first hedgehog, should not be too different
        # from second hedgehog anyway depending on the slack
        robot_time = hedge_1_time[hedge_1_index]

    return robot_pose, robot_time


def get_hedge_pos(bag_file_path, hedge='hedge_1'):
    """
        Return position of hedghog with accompagnying time vector. Time vector is relative to experiment starting time
        
    """
    

    #   Get position data
    hedge_topic = "/" + hedge + "/hedge_pos_ang"
    hedge_msgs = bag.get_topic_data(bag_file_path, hedge_topic)
    hedge_pos = np.empty([len(hedge_msgs), 3]) # Hedgehog coordinates [x,y,z]
    hedge_time = np.empty(len(hedge_msgs))
    
    for i in range(len(hedge_msgs)):
        hedge_msg = hedge_msgs[i]
        
        # Convert timestamp to duration in seconds since start so that it can  
        # be compared to other data from the same rosbag
        if i == 0:
            t_start = hedge_msg.timestamp_ms
        hedge_time[i] = (hedge_msg.timestamp_ms - t_start)/1000.0 
        
        hedge_pos[i][0] = hedge_msg.x_m
        hedge_pos[i][1] = hedge_msg.y_m
        hedge_pos[i][2] = hedge_msg.z_m
        
    return hedge_pos, hedge_time

def get_marker_pos(bag_file_path, z_lim=1.5):
    """
        Get marker position [x, y, z] with timestamp in unix time
    """
    
    marker_msgs = bag.get_topic_data(bag_file_path, "/visualization_marker")
    marker_pos = [[],[],[]] # Marker coordinates [x,y,z]
    marker_time = np.array([])
    
    for marker_msg in marker_msgs:        
        # Reject positions with large z coordinates as these are likely the 
        # beacons (z= 2.8)
        
        if marker_msg.pose.position.z < z_lim:

            marker_time = np.append(marker_time, marker_msg.header.stamp.to_sec())
            marker_pos[0].append(marker_msg.pose.position.x)
            marker_pos[1].append(marker_msg.pose.position.y)
            marker_pos[2].append(marker_msg.pose.position.z)

    marker_pos = np.asarray(marker_pos)
            
    return marker_pos, marker_time

def get_beacon_pos(bag_file_path):
    """!
    @param bag_file_path: full path to rosbag file
    @return beacon_pos: beacon locations in a dictionary containing the x, y and z coordinates per beacon address
    """

    beacon_msgs = bag.get_topic_data(bag_file_path, "/hedge_1/beacons_pos_a")
    beacon_pos = {}
    for beacon_msg in beacon_msgs:
        if beacon_msg.address not in beacon_pos.keys():  # Only add beacons not yet present in dictionary
            beacon_pos[beacon_msg.address] = [beacon_msg.x_m, beacon_msg.y_m, beacon_msg.z_m]

    return beacon_pos

    
if __name__=="__main__":
    
    import matplotlib.pyplot as plt
    
    #   Select files with GUI
    bagFilePath = get_file_path("Select .bag file").name   
    hedge_pos_1 = get_hedge_pos(bagFilePath)
    hedge_pos_2 = get_hedge_pos(bagFilePath, hedge='hedge_2')

    fig = plt.figure(1)
    fig.clf()
    plt.plot(hedge_pos_1[:,0], hedge_pos_1[:,1], label='hedge_1_pos')
    plt.plot(hedge_pos_2[:,0], hedge_pos_2[:,1], label='hedge_2_pos')
    plt.xlim([min(hedge_pos_1[:,0])-1, max(hedge_pos_1[:,0])+1])
    plt.ylim([min(hedge_pos_1[:,1])-1, max(hedge_pos_1[:,1])+1])
    plt.grid()
    plt.legend()
