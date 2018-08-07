#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 31 10:36:20 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Functions for postprocessing data from HTC VIVE VR tracking system. Was 
originally part of verification.py file. This version is currently untested due
to the lack of available data.s

"""
import numpy as np

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
    