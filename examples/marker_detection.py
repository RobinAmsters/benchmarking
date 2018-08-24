#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May  9 16:19:59 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Example aruco marker detection postprocessing file

"""
# Include parent directory in pythonpath
import os, sys, inspect
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from file_select_gui import get_file_path
from webcam import get_webcam_reference


def track_marker():
    save_figs = False  # Save the resulting plots

    # Define marker parameters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(3, 3, .07, .035, dictionary)
    marker_size = 0.10  # In meter

    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)

    #       Select files with GUI
    cam_params_file = get_file_path("Select camera parameters file").name
    video_file = get_file_path("Select video file").name

    # cam_params_file = '/home/quinten/Documents/vakantiejob/camera/benchmarking/example_data/cam_params.pckl'
    # video_file = '/home/quinten/Documents/vakantiejob/camera/benchmarking/example_data/robot_aruco.webm'

    all_tvec, all_rvec = get_webcam_reference(video_file, cam_params_file, dictionary,
                                              marker_size, board, show_video=True,
                                              save_output=False, output_file_name='example.avi')

    print(all_tvec.shape)

    (X, Y, Z) = project_in_main_plane(all_tvec)

    fig = plt.figure(3)
    fig.clf()
    ax = fig.gca(projection='3d')
    ax.axis('equal')
    ax.scatter(X, Y)


    visualize(all_tvec, saveFig=save_figs)
    write_to_csv(np.column_stack((X, Y, Z)), '/tmp/test_vid', 0, 30)


def project_in_main_plane(matrix):
    meanVector = np.mean(matrix, axis=0)
    variance = matrix - meanVector
    (u, s, v) = np.linalg.svd(variance)
    xVec = v[0]
    yVec = v[1]
    zVec = v[2]
    xVec.shape = (1, 3)
    yVec.shape = (1, 3)
    zVec.shape = (1, 3)
    X = np.dot(variance, xVec.transpose())
    Y = np.dot(variance, yVec.transpose())
    Z = np.dot(variance, zVec.transpose())
    return (X, Y, Z)


def write_to_csv(posVec, fileName, startTime, framerate):
    posVec = 1000 * posVec
    with open(fileName, 'w') as outputFile:
        writer = csv.writer(outputFile)
        header = ['timestamp', 'X', 'Y', 'Z']
        writer.writerow(header)
        for i in range(posVec.shape[0]):
            writer.writerow([startTime + float(i)/framerate] + list(posVec[i]))


def visualize(posVec, plotInterval=1, saveFig=False, fontSize=20):
    #             PLOTTING

    fig_name = 'Robot position'
    fig1 = plt.figure(1)
    fig1.clf()
    fig1.canvas.set_window_title(fig_name)
    ax1 = fig1.gca(projection='3d')
    ax1.axis('equal')
    ax1.scatter3D(posVec[:, 0], posVec[:, 1], posVec[:, 2], label='Camera measurements')
    ax1.set_xlim(min(posVec[:, 0]) - plotInterval, max(posVec[:, 0]) + plotInterval)
    ax1.set_ylim(min(posVec[:, 1]) - plotInterval, max(posVec[:, 1]) + plotInterval)
    ax1.set_zlim(min(posVec[:, 2]) - plotInterval, max(posVec[:, 2]) + plotInterval)
    ax1.set_title('Robot position', fontsize=fontSize, y=1.05)
    plt.tick_params(axis='both', which='major', labelsize=fontSize)

    if saveFig:
        plt.savefig((fig_name + '.png'), dpi=300, bbox_inches='tight')
        plt.savefig((fig_name + '.eps'), format='eps', dpi=300.0, bbox_inches='tight')

    fig_name = 'Individual coordinates'
    fig = plt.figure(2)
    fig.clf()
    fig.canvas.set_window_title(fig_name)

    ax1 = fig.add_subplot(131)
    ax1.plot(posVec[:, 0], label='camera x coordinate')
    ax1.set_ylim(min(posVec[:, 0]) - plotInterval, max(posVec[:, 0]) + plotInterval)
    ax1.set_xlabel('Time [?]', fontsize=fontSize)
    ax1.set_ylabel('X [m]', fontsize=fontSize)
    plt.grid()

    ax2 = fig.add_subplot(132)
    ax2.plot(posVec[:, 1], label='camera y coordinate')
    ax2.set_ylim(min(posVec[:, 1]) - plotInterval, max(posVec[:, 1]) + plotInterval)
    ax2.set_xlabel('Time [?]', fontsize=fontSize)
    ax2.set_ylabel('Y [m]', fontsize=fontSize)
    plt.grid()

    ax3 = fig.add_subplot(133)
    ax3.plot(posVec[:, 2], label='camera z coordinate')
    ax3.set_ylim(min(posVec[:, 2]) - plotInterval, max(posVec[:, 2]) + plotInterval)
    ax3.set_xlabel('Time [?]', fontsize=fontSize)
    ax3.set_ylabel('Z [m]', fontsize=fontSize)
    plt.grid()

    if saveFig:
        plt.savefig((fig_name + '.png'), dpi=300, bbox_inches='tight')
        plt.savefig((fig_name + '.eps'), format='eps', dpi=300.0, bbox_inches='tight')
    plt.draw()


if __name__ == '__main__':
    track_marker()
    plt.show()
