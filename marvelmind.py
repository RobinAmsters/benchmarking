#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 31 11:49:26 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be
@brief: postprocess data from marvelmind ultrasound positioning system
@details: functions assume data was recorded in a rosbag file

"""
import math
import copy
import sympy
import bag_data as bag
import numpy as np
from file_select_gui import get_file_path
from scipy.optimize import root



def rotate_point(origin, point, angle):
    """!
        @brief Rotate a point counterclockwise by a given angle around a given origin (in 2D)

        @param origin: origin around which to rotate the point [x, y]
        @param point: point to rotate  [x, y]
        @param angle: rotation angle in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)

    return qx, qy


def transform_to_global(all_pos, beacon_origin):
    all_pos_rot_trans = np.empty([len(all_pos),2])
    for i in range(len(all_pos)):
        pos = all_pos[i]
        pos_rot = rotate_point([0.0, 0.0], [pos[0], pos[1]], np.radians(-45))
        pos_rot_trans = [pos_rot[0] - beacon_origin[0], pos_rot[1] - beacon_origin[1]]
        all_pos_rot_trans[i] = pos_rot_trans

    return all_pos_rot_trans

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

    # should match shortest list to longest list, otherwhise the approach below generates duplicate matches
    swap = False
    if len(time_2) > len(time_1):
        time_1_copy = copy.deepcopy(time_1)
        time_1 = time_2
        time_2 = time_1_copy
        swap = True

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

    # In case we swapped the time vectors, the indices are reversed as well
    if swap:
        return index_2, index_1
    else:
        return index_1, index_2

def get_robot_pose_old(hedge_1_pos, hedge_1_time, hedge_2_pos, hedge_2_time, hedge_positions, slack=0.1):
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
        theta = get_vector_angle(hedge_1_pos[hedge_1_index[i]], hedge_2_pos[hedge_2_index[i]])
        # @todo identify hedge adress somehow
        robot_pose[i, 0] = x_hedhe_g + x_hedhe_r*np.sin(theta) - y_hedhe_r*np.cos(theta)
        robot_pose[i, 1] = y_hedhe_g - y_hedhe_r*np.sin(theta) - y_hedhe_r*np.cos(theta)

        # Get angle between hedgehogs, to be used as ground truth for robot heading
        robot_pose[i, 2] = theta

        # Take timestamps of robot pose the same as aligned timestamps of first hedgehog, should not be too different
        # from second hedgehog anyway depending on the slack
        robot_time = hedge_1_time[hedge_1_index]

    return robot_pose, robot_time

def get_robot_pose(pos, time, origin, params, method='odom', d_hedge_upper=0.18, d_hedge_lower=0.23, tol=1.0):
    """!
    @brief calculate robot pose based marvelmind and optionally odometry measurements
    @details calculation is performed either by using the coordinates of two hedgehogs (method='hedge'), or by using the
    coordinates of 1 hedgehog and odometry angles (method='odom'). All coordinates should first be expressed in a
    common world coordinate frame
    @param pos: coordinates of hedgehogs as a dictionary, in the world coordinate frame.
    @param: time stamps corresponding to the coordinates in pos. Should also be a dictionary with the same keys as pos
    @param params: YAML parameters (should at least contain hedgehog positions)
    @param method: calculation method for robot pose. Accepted values are 'odom' and 'hedge'
    @param d_hedge_upper: upper limit on distance between hedgehogs. Above this limit, robot pose wil be returned as NAN
    @param d_hedge_lower: lower limit on distance between hedgehogs. Above this limit, robot pose wil be returned as NAN
    @param tol: tolerance for numerical solvers
    @return robot_pose: pose of mobile robot [[x, y, theta]]
    """
    # Get hedgehog ids from YAML parameters
    hedge_id = params['hedge_positions'].keys()
    id_0 = hedge_id[0]
    id_1 = hedge_id[1]

    # Get position of first hedgehog relative to robot center
    x_hedge_0_rob = params['hedge_positions'][id_0][0]
    y_hedge_0_rob = params['hedge_positions'][id_0][1]

    # Calculate robot pose from 1 hedgehog coordinate and odometry angle
    if method=='odom':

        # Get odometry and hedgehog coordinates
        odom_pos = pos['odom']
        odom_time = time['odom']
        hedge_pos = pos['hedge'][id_0]
        hedge_time = time['hedge'][id_0]

        # Initialize collection
        robot_pose = np.empty([len(hedge_pos), 3])

        # only take timestamps that approximately match
        hedge_index, odom_index = align_time_index(hedge_time, odom_time, slack=0.1)
        hedge_time = hedge_time[hedge_index]
        hedge_pos = hedge_pos[hedge_index]
        for i in range(len(odom_pos)):
            odom_pos[i] = odom_pos[i][odom_index]

        # Calculate robot center pose, assume odometry angle is equal to robot angle in the world frame
        for i in range(len(odom_pos[2])):
            theta = odom_pos[2][i]
            # TODO translate the hedge angle to world frame first
            robot_pose[i][0] = hedge_pos[i][0] - x_hedge_0_rob * np.cos(theta) + y_hedge_0_rob * np.sin(theta)
            robot_pose[i][1] = hedge_pos[i][1] - x_hedge_0_rob * np.sin(theta) - y_hedge_0_rob * np.cos(theta)
            robot_pose[i][2] = theta

    # Calculate robot_pose from coordinates of 2 hedghehogs
    elif method=='hedge_pos':

        hedge_pos = pos['hedge']
        hedge_time = time['hedge']

        # Positions of second hedgehog relative to robot center
        x_hedge_1_rob = params['hedge_positions'][id_1][0]
        y_hedge_1_rob = params['hedge_positions'][id_1][1]

        # symbolic representation for sympy solver
        x_rob, y_rob, theta = sympy.symbols('x_rob, y_rob, theta')

        # Synchronize timestamps of both hedgehogs
        hedge_0_index, hedge_1_index = align_time_index(hedge_time[id_0], hedge_time[id_1], slack=0.1) # only take timestamps that approximately match
        hedge_index = {id_0:hedge_0_index, id_1:hedge_1_index}
        for id in hedge_id:
            hedge_pos[id] = hedge_pos[id][hedge_index[id]]
            hedge_time[id] = hedge_time[id][hedge_index[id]]

        # Initialize collections
        robot_pose = np.empty([len(hedge_pos[id_0]), 3])
        pose_prev = np.zeros(3)
        d_hedge = []



        # Use non linear solver to calculate robot pose from hedgehog coordinates and their position relative to the
        # robot center
        for i in range(len(hedge_pos[id_0])):

            x_hedge_0 = hedge_pos[id_0][i][0]
            y_hedge_0 = hedge_pos[id_0][i][1]
            x_hedge_1 = hedge_pos[id_1][i][0]
            y_hedge_1 = hedge_pos[id_1][i][1]

            data = (x_hedge_0, y_hedge_0, x_hedge_1, y_hedge_1, x_hedge_0_rob, y_hedge_0_rob, x_hedge_1_rob, y_hedge_1_rob)
            d_hedge_i = np.sqrt((x_hedge_0 - x_hedge_1) ** 2 + (y_hedge_0 - y_hedge_1) ** 2) # Distance between hedgehogs, should be a constant value during an experiment

            # Only attempt a solution if the distance between hedgehogs is within set limits
            if (d_hedge_i >= d_hedge_upper and d_hedge_i <= d_hedge_lower):
                sol = root(hedge_to_robot, pose_prev, args=data, method='lm')
                sol = sol.x
                d_hedge.append(d_hedge_i)
                pose_prev[0:3] = sol[0:3]
                robot_pose[i] = sol[0:3]
            else:
                d_hedge.append(np.nan)
                robot_pose[i] = [np.nan, np.nan, np.nan]
    else:
        raise Exception("Invalid method, accepted values are 'hedge_pos', 'odom'")

    return robot_pose

def hedge_to_robot(vars, *data):
    """!
    @brief equations that define the relation between global hedgehog coordinates, and global robot pose.
    @param vars: symbolic variables of the robot pose (x_rob_global, y_rob_global, theta )
    @return data: coordinates of the hedgehogs in global and robot coordinate frames
    (x_hedge_1_global, y_hedge_1_global, x_hedge_2_global,y_hedge_2_global, x_hedge_1_rob, y_hedge_1_rob, x_hedge_2_rob, y_hedge_2_rob)
    """
    x_rob_global, y_rob_global, theta = vars
    x_hedge_1_global, y_hedge_1_global, x_hedge_2_global,y_hedge_2_global, x_hedge_1_rob, y_hedge_1_rob, x_hedge_2_rob, y_hedge_2_rob = data

    f1 = -x_hedge_1_global + x_rob_global + x_hedge_1_rob*sympy.cos(theta) - y_hedge_1_rob*sympy.sin(theta)
    f2 = -y_hedge_1_global + y_rob_global + x_hedge_1_rob*sympy.sin(theta) + y_hedge_1_rob*sympy.cos(theta)
    f3 = -x_hedge_2_global + x_rob_global + x_hedge_2_rob*sympy.cos(theta) - y_hedge_2_rob*sympy.sin(theta)
    f4 = -y_hedge_2_global + y_rob_global + x_hedge_2_rob*sympy.sin(theta) + y_hedge_2_rob*sympy.cos(theta)

    return (f1, f2, f3, f4)

def get_multi_hedge_pos(bag_file_path, hedge_address=[17, 59], hedge_names=['hedge_1','hedge_2']):

    hedge_pos = {}
    hedge_rot = {}
    hedge_time = {}
    for id in hedge_address:
        hedge_pos[id] = []
        hedge_rot[id] = []
        hedge_time[id] = []

    #   Get position data from all topics for all hedgehogs
    all_hedge_msgs = []
    for hedge in hedge_names:
        hedge_topic = "/" + hedge + "/hedge_pos_ang"
        hedge_msgs = bag.get_topic_data(bag_file_path, hedge_topic)
        all_hedge_msgs = np.append(all_hedge_msgs, hedge_msgs)

    # Seperate data based on ids
    for i in range(len(all_hedge_msgs)):
        hedge_msg = all_hedge_msgs[i]
        id = hedge_msg.address
        # Convert timestamp to duration in seconds since start so that it can
        # be compared to other data from the same rosbag
        if i == 0:
            t_start = hedge_msg.timestamp_ms

        hedge_time[id].append((hedge_msg.timestamp_ms - t_start) / 1000.0)
        hedge_pos[id].append([hedge_msg.x_m, hedge_msg.y_m, hedge_msg.z_m])
        hedge_rot[id].append(np.radians(hedge_msg.angle))

    # Convert to numpy arrays for easier indexing
    t_0 = []
    for id in hedge_pos.keys():
        index = np.argsort(hedge_time[id]) # sort based on timestamps
        hedge_pos[id] = np.asarray(hedge_pos[id])[index]
        hedge_time[id] = np.asarray(hedge_time[id])[index]
        hedge_rot[id] = np.asarray(hedge_rot[id])[index]
        t_0.append(hedge_time[id][0])

    # If the first entry of the time vector is zero, the timestamps started in another timevector. So shift the every vector by this amount
    time_shift = abs(min(t_0))
    for id in hedge_pos.keys():
        hedge_time[id] = hedge_time[id] + time_shift

    return hedge_pos, hedge_time, hedge_rot


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
