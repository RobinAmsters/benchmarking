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
from file_select_gui import get_file_path
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# ==============================================================================
def get_topic_data(bagFile, topic, return_t=False):
    """
        Return all messages from a specific topic
    """

    all_msg = []
    if return_t:
        all_t = []


    # Initialize rosbag object
    bag = rosbag.Bag(bagFile)

    for topic, msg, t in bag.read_messages(topics=[topic]):
        all_msg = np.append(all_msg, msg)
        if return_t:
            all_t = np.append(all_t, t.to_sec())

    if return_t:
        return all_msg, all_t
    else:
        return all_msg


# ==============================================================================

def get_joint_data(bagFile, joint_name, duration=True):
    """
        Function that filters bag files to obtain data from a joint that is 
        published to the /tf topic.
        
        Only x-, y- and theta-coordinates are returned
        
        if convert_to_sec is set to True (default), then the first timestamp 
        will be taken as zero seconds, and the following timesteps that are 
        retured will be relative to this timestamp.
        
    """
    # INITIALIZATION
    x = np.array([])
    y = np.array([])
    theta = np.array([])
    all_t = np.array([])
    bag = rosbag.Bag(bagFile)  # Initialize rosbag object
    first = True  # True on first iteration to take first timestamp

    # Add message values to collections
    for topic, msg, t in bag.read_messages(topics=['/tf']):

        joint = msg.transforms[0].child_frame_id
        translation = msg.transforms[0].transform.translation
        orientation = msg.transforms[0].transform.rotation
        euler = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        if joint == joint_name:

            # Get timestamp in seconds
            t = msg.transforms[0].header.stamp
            t_sec = t.to_sec()
            if duration:
                if first:
                    t_0 = t_sec
                    first = False

                all_t = np.append(all_t, t_sec - t_0)

            else:
                all_t = np.append(all_t, t_sec)

            # Get x, y and z coordinates

            x = np.append(x, translation.x)
            y = np.append(y, translation.y)
            theta = np.append(theta, euler[2])

    pose = [x, y, theta]

    return pose, all_t


# ==============================================================================
#           DEMO
# ==============================================================================
if __name__ == "__main__":

    # Open bagfile and obtain robot position
    bagFile = get_file_path('Select bag file').name
    pose, all_t = get_joint_data(bagFile, 'base_footprint')

    # Get laptop charge data from rostopic
    charge_msgs = get_topic_data(bagFile, '/laptop_charge')
    t_charge = []
    voltage_charge = []
    first = True
    for charge_msg in charge_msgs:
        # Get timestamp in seconds
        t = charge_msg.header.stamp
        t_sec = t.to_sec()

        # Take first timestep as 0 seconds
        if first:
            t_0 = t_sec
            first = False

        # Add data to collections
        t_charge = np.append(t_charge, t_sec - t_0)
        voltage_charge = np.append(voltage_charge, charge_msg.voltage)

    # Plotting
    import matplotlib.pyplot as plt

    fig = plt.figure(1)

    # Plot robot pose
    ax = fig.add_subplot(211)
    ax.scatter(pose[0], pose[1], color='black', s=5, label='robot pose')
    ax.set_xlabel('X-coordinate [m]')
    ax.set_ylabel('Y-coordinate [m]')
    plt.legend()
    plt.grid()

    # Plot laptop charge
    ax = fig.add_subplot(212)
    ax.plot(t_charge, voltage_charge, color='black', label='laptop charge')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Voltage [V]')
    plt.legend()
    plt.grid()

    plt.show()
