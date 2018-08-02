#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 4 14:10:11 2017

@author: Robin Amsters

File to postprocess data from experiments. Position estimations of the robot
(from .bag files) are compared to position estimations from the krypton K600
coordinate measuring machine (.mat files)

TODO:
    - cleanup file

"""
import math
import bag_data as bag
import marvelmind as marvelmind

import numpy as np
import matplotlib.pyplot as plt
import vlp_classes as vlp

from file_select_gui import get_file_path
from scipy.signal import butter, lfilter, freqz

plt.rc('text', usetex=False)

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
#               OTHER FUNCTIONS       
#==============================================================================
def get_light_intensities(bagFilePath, duration=True):
    light_msgs = bag.get_topic_data(bagFilePath, "/light_intensity")
    intensities = np.empty([len(light_msgs), 5])
    light_time = np.empty([len(light_msgs)])
    
    for i in range(len(light_msgs)):
        light_msg = light_msgs[i]
        intensities[i] = light_msg.intensity
        
        if duration:
            if i == 0:
                t_start = light_msg.header.stamp
                
            t = light_msg.header.stamp - t_start
        else:
            t = light_msg.header.stamp
            
        light_time[i] = t.to_sec()
        
    return intensities, light_time

def get_dr(all_x_est, all_y_est, all_x_ref, all_y_ref):
    
    dr = np.ones(len(all_x_est))
    
    for i in range(len(all_x_est)):
        x_est = all_x_est[i]
        y_est = all_y_est[i]
        x_ref = all_x_ref[i]
        y_ref = all_y_ref[i]
        
        dr[i] = math.sqrt((x_ref-x_est)**2 + (y_ref-y_est)**2)
        
    return dr

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y
        
#==============================================================================
#               GET AND PLOT DATA
#==============================================================================

if __name__ == "__main__":
#%%                   GET DATA FROM FILES

    rx_i = 1 # Number of receivers to process, set to 5 for all receivers

    #   Select files with GUI
#    bagFilePath = get_file_path("Select .bag file").name    
    bagFilePath = "/home/robin/catkin_ws/src/benchmarking/example_data/UVLP_path_1.bag"
    
    #   PROCESS LIGHT MEASUREMENTS
    intensities, light_time = get_light_intensities(bagFilePath, duration=False)
    
    # Reduce noise and outliers with butterworth low pass filter
    intensities_filtered = np.empty_like(intensities)
    for i in range(rx_i):
        intensities_filtered[:,i] = butter_lowpass_filter(intensities[:,i], 1, len(intensities)/(light_time[-1]-light_time[0]), order=1)
        
    #   HEDGEHOG POSITIONS
    hedge_1_pos, hedge_1_time = marvelmind.get_hedge_pos(bagFilePath, hedge='hedge_1')
    hedge_2_pos, hedge_2_time = marvelmind.get_hedge_pos(bagFilePath, hedge='hedge_2')
#%%                   VERIFIFY MODEL    
    room = vlp.Room() 
# TODO: add real LED positions from beacon locations
    intensity_model = room.predict_measurement_xy(hedge_1_pos[:,0], hedge_1_pos[:,1], estimated_params={})
    print intensity_model
    
#%%                   CALCULATE POSITIONING ERROR
       
#%%                   PLOTTING
            
    fig = plt.figure(1)        
    fig.clf()
    plt.title('Light intensity measurements')
    plt.xlabel('Time [s]')
    plt.ylabel('ADC voltage [V]')
    for i in range(rx_i):
        plt.plot(time, intensities[:,i], label='Receiver ' + str(i+1)) 
        plt.plot(time, intensities_filtered[:,i], label='Receiver ' + str(i+1) + ' filtered') 
    plt.plot(hedge_1_time, intensity_model[2], label='Model')
    plt.xlim([0.0, time[-1]])
    plt.ylim([0.0, 5.0])
    plt.legend()
    plt.grid()
    plt.show()
