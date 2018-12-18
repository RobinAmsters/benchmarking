#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 16 11:11:32 2017

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Filter bag files into a new bag file that only contains the topics specified.
For now, the desired topics have to specified in this file

TODO: 
    - allow user to specify topics more easily

"""

import os
import sys

from fnmatch import fnmatch
from file_select_gui import get_directory_path, get_file_path

import numpy as np

def get_all_bag_files_in_directory(directory):
    """
        Function that returns all the path to all .bag files in a directory
        and its subdirectories
        
    """
    
    root = directory
    pattern = "*.bag"
    
    bag_files = np.array([])
    
    for path, subdirs, files in os.walk(root):
        for name in files:
            if fnmatch(name, pattern):
                bag_files = np.append(bag_files, os.path.join(path, name))
    
    return bag_files

def filter_bag_file(bag_path, topics):
    """
        Function that filters all topics in a single bag file except the ones
        relevant for retesing the VLP algorithm
        
    """
    
    new_bag = bag_path[0:len(bag_path) - 4] + '_filtered.bag' # Construct name of new .bag file
    
    cmd = 'rosbag filter ' + bag_path + ' ' + new_bag + ' \'topic=="' + topics[0] + '"'
    
    for i in range(1,len(topics)):
        cmd = cmd + ' or topic =="' + topics[i] + '"'
    
    cmd = cmd + '\''
    os.system(cmd)

if __name__ == "__main__":
        
    args = sys.argv
    topics = ['/light_intensity', '/mobile_base/sensors/core', '/mobile_base/sensors/imu_data',
              '/mobile_base/sensors/imu_data_raw', '/visualization_marker',
              '/hedge_1/beacons_pos_a', '/hedge_1/hedge_pos', '/hedge_1/hedge_pos_a', '/hedge_1/hedge_pos_ang',
              '/hedge_2/beacons_pos_a', '/hedge_2/hedge_pos', '/hedge_2/hedge_pos_a', '/hedge_2/hedge_pos_ang']
    
    # If no argument is passed, filter a single file
    if len(sys.argv) < 2:
        bagFile = get_file_path('Select bag file').name
        filter_bag_file(bagFile, topics)
    
    # If a valid argument is passed, filter the entire directory and all subdirectories    
    else:
        filter_arg = sys.argv[1]
        acceptable_agrs = ['dir', 'Dir', 'directory', 'Directory', 'filterdir', 'filterdirectory', 'FilterDirectory', 'filter_dir', 'filter_directory']
        
        if filter_arg in acceptable_agrs:
            bag_dir = get_directory_path('Select directory of .bag files to filter (SUBDIRECTORIES WILL BE FILTERED TOO)')     # Select directory via GUI
            bag_files = get_all_bag_files_in_directory(bag_dir)                                                                    # Get all bag files in a directory
            
            for bag_file in bag_files:                                                                                        # Filter all bag files
                print("Processing file: " + bag_file)
                filter_bag_file(bag_file, topics)
                
        else:
            print(str(filter_arg) +  ' is not a known argument, acceptable arguments are: ' + str(acceptable_agrs))

       
