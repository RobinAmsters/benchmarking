#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 16 11:11:32 2017

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

File to filter bag files into topics neccesary for testing the VLP algorithm

Topics that are present in the new bag file:
	- /light_intensity
	- /mobile_base/sensors/core
	- /mobile_base/sensors/imu_data
"""

import os
import sys

from fnmatch import fnmatch
from FileSelectGui import getDirectoryPath, getFilePath

import numpy as np

def getAllBagFilesInDirectory(directory):
    """
        Function that returns all the path to all .bag files in a directory
        and its subdirectories
        
    """
    
    root = directory
    pattern = "*.bag"
    
    bagFiles = np.array([])
    
    for path, subdirs, files in os.walk(root):
        for name in files:
            if fnmatch(name, pattern):
                bagFiles = np.append(bagFiles, os.path.join(path, name))
    
    return bagFiles

def filterBagFile(bagPath):
    """
        Function that filters all topics in a single bag file except the ones
        relevant for retesing the VLP algorithm
        
    """
    
    topics = ['/light_intensity', '/mobile_base/sensors/core', '/mobile_base/sensors/imu_data', '/scan'] # Topics to include in filtered bag file
    
    new_bag = bagPath[0:len(bagPath)-4] + '_filtered.bag' # Construct name of new .bag file
    
    cmd = 'rosbag filter ' + bagPath + ' ' + new_bag + ' \'topic=="' + topics[0] + '"'
    
    for i in range(1,len(topics)):
        cmd = cmd + ' or topic =="' + topics[i] + '"'
    
#    cmd = cmd + ' or topic == "/tf" and m.transforms[0].header.frame_id == "odom" and m.transforms[0].child_frame_id == "base_footprint"'
#    cmd = cmd + '\''
#    cmd = cmd + ' or topic == "/tf_static"'
    cmd = cmd + '\''
#    print(cmd)
    os.system(cmd)

if __name__ == "__main__":
    
    # If no argument is passed, filter a single file
    args = sys.argv
    
    if len(sys.argv) < 2:
        bagFile = getFilePath('Select bag file').name
        filterBagFile(bagFile)  
    # If a valid argument is passed, filter the entire directory and all subdirectories    
    else:
        filter_arg = sys.argv[1]
        acceptable_agrs = ['dir', 'Dir', 'directory', 'Directory', 'filterdir', 'filterdirectory', 'FilterDirectory', 'filter_dir', 'filter_directory']
        
        if filter_arg in acceptable_agrs:
            bagDir = getDirectoryPath('Select directory of .bag files to filter (SUBDIRECTORIES WILL BE FILTERED TOO)')     # Select directory via GUI
            bagFiles = getAllBagFilesInDirectory(bagDir)                                                                    # Get all bag files in a directory
            
            for bagFile in bagFiles:                                                                                        # Filter all bag files
                print("Processing file: " + bagFile)
                filterBagFile(bagFile)
                
        else:
            print(str(filter_arg) +  ' is not a known argument, acceptable arguments are: ' + str(acceptable_agrs))

       
