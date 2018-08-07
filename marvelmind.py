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


def get_hedge_pos(bagFilePath, hedge='hedge_1'):
    """
        Return position of hedghog with accompagnying time vector.
        
    """
    

    #   Get position data
    hedge_topic = "/" + hedge + "/hedge_pos"
    hedge_msgs = bag.get_topic_data(bagFilePath, hedge_topic)
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

def get_marker_pos(bagFilePath, z_lim=1.5):
    """
        Get marker position with timestamp in unix time
    """
    
    marker_msgs = bag.get_topic_data(bagFilePath, "/visualization_marker")
    marker_pos = [[],[],[]] # Marker coordinates [x,y,z]
    marker_time = []
    
    for marker_msg in marker_msgs:        
        # Reject positions with large z coordinates as these are likely the 
        # beacons (z= 2.8)
        
        if marker_msg.pose.position.z < z_lim:
        
            marker_time.append(marker_msg.header.stamp.to_sec())
            marker_pos[0].append(marker_msg.pose.position.x)
            marker_pos[1].append(marker_msg.pose.position.y)
            marker_pos[2].append(marker_msg.pose.position.z)
            
    return marker_pos, marker_time

    
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
