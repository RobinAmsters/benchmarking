#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 11:05:28 2017

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

General purpose file for obtaining data from rosbag files

"""
import rosbag

import numpy as np

from FileSelectGui import getFilePath

#==============================================================================
def get_topic_data(bagFile, topic):
    """
        Return all messages from a specific topic
    """
    
    all_msg = []
    
    # Initialize rosbag object
    bag = rosbag.Bag(bagFile)
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        
        all_msg = np.append(all_msg, msg)

    return all_msg
#==============================================================================

def get_joint_data(bagFile, joint_name):
    """
        Function that filters bag files to obtain data from a joint that is 
        published to the /tf topic.
        
        Only x-, y- and z-coordinates are returned
        
    """
    x = np.array([])
    y = np.array([])
    z = np.array([])
    
    all_t = np.array([])
    
    # Initialize rosbag object
    bag = rosbag.Bag(bagFile)
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        
        joint = msg.transforms[0].child_frame_id
        translation = msg.transforms[0].transform.translation
        
        if joint == joint_name: 

            # Get timestamp in seconds
            t = msg.transforms[0].header.stamp
            t_sec = t.to_sec()
            all_t = np.append(all_t, t_sec)
            
            # Get x, y and z coordinates
            pose = [translation.x , translation.y, translation.z]

            x = np.append(x, pose[0])
            y = np.append(y, pose[1])
            z = np.append(z, pose[2])
    
    pose = [x,y,z]
    
    return pose, all_t

#==============================================================================

def get_robot_pose(bagFile):
    """
        Function that gets robot pose (position, orientation) from bag files.
        Robot frame should be named 'base_footprint'.
        Initial position is remapped to the origin
    """
    
    
    x = np.array([])
    y = np.array([])
    z = np.array([])
    th = np.array([])
    
    robot_t = np.array([])
    first = 1
    
    bag = rosbag.Bag(bagFile)
    
    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        
        joint = msg.transforms[0].child_frame_id.split('_')
        
        if joint[0] == 'base' and joint[1] == 'footprint': 
            
            # Get time difference from first timestamp
            if first:
                t_start = msg.transforms[0].header.stamp
                translation = msg.transforms[0].transform.translation
                rotation = msg.transforms[0].transform.rotation
                euler = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
                
                x_0 = translation.x
                y_0 = translation.y
                z_0 = translation.z
                th_0 = 0
                
                first = 0
            
            else:
                
                translation = msg.transforms[0].transform.translation
                rotation = msg.transforms[0].transform.rotation
                euler = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            
                t = msg.transforms[0].header.stamp - t_start
                t = t.to_sec()
                robot_t = np.append(robot_t, t)
                
                x = np.append(x, translation.x - x_0)
                y = np.append(y, translation.y - y_0)
                z = np.append(z, translation.z - z_0)
                th = np.append(th, euler[2] - th_0)
    
    robot_pose = [x, y, z, th]
    
    return robot_pose, robot_t

#==============================================================================
#           DEMO
#==============================================================================
if __name__ == "__main__":
    
    # Open bagfile and obtain robot position
    bagFile = getFilePath('Select bag file').name
    robot_pose, robot_t = get_robot_pose(bagFile)
    
    # Plotting
    import matplotlib.pyplot as plt
    
    plt.scatter(robot_pose[0], robot_pose[1], color='black', s=5, label='Robot position')
    plt.xlabel('X-coordinate [m]')
    plt.ylabel('Y-coordinate [m]')
    plt.legend()
    plt.grid()
    
    plt.show()