#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 31 11:49:26 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be



"""

import bag_data as bag
import numpy as np

from file_select_gui import get_file_path


def get_hedge_pos(bagFilePath, hedge='hedge_1'):

    #   Get position data
    hedge_topic = "/" + hedge + "/hedge_pos"
    hedge_msgs = bag.get_topic_data(bagFilePath, hedge_topic)
    hedge_pos = np.empty([len(hedge_msgs), 3]) # Hedgehog coordinates [x,y,z]
    
    for i in range(len(hedge_msgs)):
        hedge_msg = hedge_msgs[i]
        hedge_pos[i][0] = hedge_msg.x_m
        hedge_pos[i][1] = hedge_msg.y_m
        hedge_pos[i][2] = hedge_msg.z_m
        
    return hedge_pos

if __name__=="__main__":
    
    import matplotlib.pyplot as plt
    
    #   Select files with GUI
    bagFilePath = get_file_path("Select .bag file").name   
    hedge_pos_1 = get_hedge_pos(bagFilePath)
    hedge_pos_2 = get_hedge_pos(bagFilePath, hedge='hedge_2')

    fig = plt.figure(1)
    fig.clf()
    plt.plot(hedge_pos_1[:,0], hedge_pos_1[:,1], label='hedge_1_pos')
    plt.plot(hedge_pos_2[:,0], hedge_pos_2[:,1], label='hedge_2_pos')
    plt.xlim([min(hedge_pos_1[:,0])-1, max(hedge_pos_1[:,0])+1])
    plt.ylim([min(hedge_pos_1[:,1])-1, max(hedge_pos_1[:,1])+1])
    plt.grid()
    plt.legend()
