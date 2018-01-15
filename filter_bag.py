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

def filterBagFile(bagPath, topics):
    """
        Function that filters all topics in a single bag file except the ones
        relevant for retesing the VLP algorithm
        
    """
    
    new_bag = bagPath[0:len(bagPath)-4] + '_filtered.bag' # Construct name of new .bag file
    
    cmd = 'rosbag filter ' + bagPath + ' ' + new_bag + ' \'topic=="' + topics[0] + '"'
    
    for i in range(1,len(topics)):
        cmd = cmd + ' or topic =="' + topics[i] + '"'
    
    cmd = cmd + '\''
    os.system(cmd)

if __name__ == "__main__":
        
    args = sys.argv
    topics = ['/light_intensity', '/mobile_base/sensors/core', '/mobile_base/sensors/imu_data', '/scan']
    
    # If no argument is passed, filter a single file
    if len(sys.argv) < 2:
        bagFile = getFilePath('Select bag file').name
        filterBagFile(bagFile, topics)  
    
    # If a valid argument is passed, filter the entire directory and all subdirectories    
    else:
        filter_arg = sys.argv[1]
        acceptable_agrs = ['dir', 'Dir', 'directory', 'Directory', 'filterdir', 'filterdirectory', 'FilterDirectory', 'filter_dir', 'filter_directory']
        
        if filter_arg in acceptable_agrs:
            bagDir = getDirectoryPath('Select directory of .bag files to filter (SUBDIRECTORIES WILL BE FILTERED TOO)')     # Select directory via GUI
            bagFiles = getAllBagFilesInDirectory(bagDir)                                                                    # Get all bag files in a directory
            
            for bagFile in bagFiles:                                                                                        # Filter all bag files
                print("Processing file: " + bagFile)
                filterBagFile(bagFile, topics)
                
        else:
            print(str(filter_arg) +  ' is not a known argument, acceptable arguments are: ' + str(acceptable_agrs))

       
