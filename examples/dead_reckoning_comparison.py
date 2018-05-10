#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 14:10:11 2017

@author: Robin Amsters

File to postprocess data from experiments. Position estimations of UVLP are compared to position estimations from dead reckoning estimates

"""

import math
import sys

import numpy as np
import matplotlib.pyplot as plt

from file_select_gui import get_file_path
from bag_data import get_joint_data

plt.rc('text', usetex=False) # Set true for latex typesetting in plots

#==============================================================================
#               OTHER FUNCTIONS       
#==============================================================================

def get_dr(all_x_est, all_y_est, all_x_ref, all_y_ref):
    """
        Get total positioning error of pose estimate relative to a reference
    """
    
    dr = np.ones(len(all_x_est))
    
    for i in range(len(all_x_est)):
        x_est = all_x_est[i]
        y_est = all_y_est[i]
        x_ref = all_x_ref[i]
        y_ref = all_y_ref[i]
        
        dr[i] = math.sqrt((x_ref-x_est)**2 + (y_ref-y_est)**2)
        
    return dr
        
#==============================================================================
#               GET AND PLOT DATA
#==============================================================================

if __name__ == "__main__":

    if len(sys.argv) == 2:
        save = bool(sys.argv[1])
        I_Tx = 1.05               # Driving current [A]
    elif len(sys.argv)  == 3:
        save = bool(sys.argv[1])
        I_Tx = float(sys.argv[2])
    else:    
        save = False    # Save the resulting plots
        I_Tx = 1.05               # Driving current [A]
        
    # Plotting parameters
    plot_interval = 2
    dpi=300
    figNum = 1
    titleSize=14
    legendSize=12
    axesSize=12
    labelSize=12
    dpi=1000
    legendLoc = 3
#%%                   GET DATA FROM FILES

    #   Select files with GUI
    bagFilePath = get_file_path("Select .bag file").name
    
    #   Get robot position estimate    
    pose_ref, t_ref = get_joint_data(bagFilePath, 'base_footprint')
    pose_est, t_est = get_joint_data(bagFilePath, 'pose')
    
    x_est = pose_est[0]
    y_est = pose_est[1]
    x_ref = pose_ref[0]
    y_ref = pose_ref[1]
    
#%%                   CALCULATE POSITIONING ERROR
    dr = get_dr(x_est, y_est, x_ref, y_ref)
       
#%%                   PLOTTING           

#==============================================================================
    # Plot position estimates
    fig_name = 'Robot position'
    fig = plt.figure(figNum)
    fig.clf()
    fig.canvas.set_window_title(fig_name)

    ax = fig.gca()
    
    ax.scatter(x_ref, y_ref, color='grey', label='Reference position')
    ax.scatter(x_est, y_est, color='black', label='VLP position estimate')

    ax.set_xlabel('X-coordinate [m]', fontsize=labelSize)
    ax.set_ylabel('Y-coordinate [m]', fontsize=labelSize)
    
    plt.legend()    
    plt.grid()

#    plt.legend(bbox_to_anchor=(0., -0.2, 1., .102), loc=legendLoc,
#           ncol=2, borderaxespad=0.0, fontsize=legendSize)
    
    if save:
        plt.savefig((bagFilePath[0:-4] + fig_name + '.eps'), format='eps', dpi=dpi,bbox_inches='tight') 
        
    figNum +=1
                
#==============================================================================
    fig_name = '_positioning_error'
    fig = plt.figure(figNum)
    fig.clf()
    fig.canvas.set_window_title(fig_name)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()

    ax = fig.gca()
    ax.scatter(t_est, dr, color='black')
    ax.set_xlabel('Time [s]', fontsize=labelSize)
    ax.set_ylabel('Positioning errror [m]', fontsize=labelSize)
    ax.set_xlim([0, max(t_est)])
    plt.grid()

    if save:
        plt.savefig((bagFilePath[0:-4] + fig_name + '.png'), format='png', dpi=dpi,bbox_inches='tight')   
    
    figNum +=1
   
    
    plt.show()
