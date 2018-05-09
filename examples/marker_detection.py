#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May  9 16:19:59 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Example aruco marker detection postprocessing file

"""
# Include parent directory in pythonpath
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from file_select_gui import get_file_path
from webcam import get_webcam_reference

fontSize = 20       # Size of labels for plots
save_figs = False  # Save the resulting plots
plot_interval = 1.0 # 


# Define marker parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(3,3,.07,.035,dictionary)
marker_size = 0.10

#       Select files with GUI
cam_params_file = get_file_path("Select camera parameters file").name
video_file = get_file_path("Select video file").name

all_tvec, all_rvec = get_webcam_reference(video_file, cam_params_file, dictionary, 
                                          marker_size, board, show_video=False, 
                                          save_output=False, output_file_name='example.avi')

#             PLOTTING
fig_name = 'Robot position'
fig1 = plt.figure(1)
fig1.clf()
fig1.canvas.set_window_title(fig_name)

ax1 = fig1.gca(projection='3d')
ax1.scatter3D(all_tvec[0], all_tvec[1], all_tvec[2], label='Camera measurements')
ax1.set_xlim(min(all_tvec[0])-plot_interval,max(all_tvec[0])+plot_interval)
ax1.set_ylim(min(all_tvec[1])-plot_interval,max(all_tvec[1])+plot_interval)
ax1.set_zlim(min(all_tvec[2])-plot_interval,max(all_tvec[2])+plot_interval)
ax1.set_title('Robot position', fontsize=fontSize, y=1.05)
plt.tick_params(axis='both', which='major', labelsize=fontSize)

if save_figs:
    plt.savefig((fig_name + '.png'), dpi=300, bbox_inches='tight')
    plt.savefig((fig_name + '.eps'), format='eps', dpi=300.0,bbox_inches='tight')

fig_name = 'Individual coordinates'
fig = plt.figure(2)
fig.clf()
fig.canvas.set_window_title(fig_name)
ax1 = fig.add_subplot(131)   
ax1.plot(all_tvec[0], label='camera x coordinate')
ax1.set_ylim(min(all_tvec[0])-plot_interval,max(all_tvec[0])+plot_interval)
ax1.set_xlabel('Time [?]', fontsize = fontSize)
ax1.set_ylabel('X [m]', fontsize = fontSize)
plt.grid()

ax2 = fig.add_subplot(132) 
ax2.plot(all_tvec[1], label='camera y coordinate')
ax2.set_ylim(min(all_tvec[1])-plot_interval,max(all_tvec[1])+plot_interval)
ax2.set_xlabel('Time [?]', fontsize = fontSize)
ax2.set_ylabel('Y [m]', fontsize = fontSize)
plt.grid()

ax3 = fig.add_subplot(133) 
ax3.plot(all_tvec[2], label='camera z coordinate')
ax3.set_ylim(min(all_tvec[2])-plot_interval,max(all_tvec[2])+plot_interval)
ax3.set_xlabel('Time [?]', fontsize = fontSize)
ax3.set_ylabel('Z [m]', fontsize = fontSize)
plt.grid()

if save_figs:
    plt.savefig((fig_name + '.png'), dpi=300, bbox_inches='tight')
    plt.savefig((fig_name + '.eps'), format='eps', dpi=300.0,bbox_inches='tight')

plt.show()
