#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May  9 16:19:59 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

TODO: test outside spyder

"""

import cv2

from file_select_gui import get_file_path
from webcam import get_webcam_reference

fontSize = 20   # Size of labels for plots
save = False    # Save the resulting plots
plot_interval = 1

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markerSize = 0.10

#       Select files with GUI
cam_params_file = get_file_path("Select camera parameters file").name
video_file = get_file_path("Select video file").name

all_tvec, all_rvec = get_webcam_reference(video_file, cam_params_file, dictionary, markerSize, show_video=True)
cv2.destroyAllWindows()