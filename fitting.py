#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 19 10:48:58 2018

@author: Robin Amsters
@email: robin.amsters@kuleuven.be
"""
import os
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats

from FileSelectGui import getDirectoryPath

def plotDist(data, mu, std, num_bins=10, figNum=1,figName='fit_results',saveFig=False):
    """
        Function to plot normal distribution fit results
        
        INPUTS:
            data=1D array containing data to fit distribution to
            mu=mean of data array
            std=standard deviation of data
            num_bins=number of bins (groups of data) for histogram
            figNum=number of figure, only needs to be changed for multiple figures
            figName=filename for the figure
            saveFig=If set to true, the figure will be saved. The user will be propmted to select the directory
        OUTPUTS:
            matplotlib figure
    """

    
    figNotSaved = "Figure not saved."
    inputError = "Incorrect input."
    
    # Make figure
    plt.figure(figNum, figsize  = (8,4))  
    plt.clf()
    
    # Plot the histograms
    n, bins, patches = plt.hist(data, num_bins, normed =True, facecolor='green', alpha=0.7, stacked = 1)

    # Plot the PDFs
    plt.figure(figNum)
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    
    # Get normaltest results
    k2, p_val = stats.normaltest(data)

    # Set figure title
    title = "Fit results: mu = %.5f,  std = %.5f, p = %.5f, k2 = %2f" % (mu, std, p_val, k2)
    plt.title(title)
    
    # Saving figure
    if saveFig:
        saveDir = getDirectoryPath("Select a folder to save to figure: ")
        savePath = saveDir + '/' + figName
        
        if os.path.isfile(savePath):
          overrideSaveString =  raw_input("File already exists, do you want to override it? (y/n): ")
          
          if type(overrideSaveString) == str:
              if overrideSaveString == 'y':
                  plt.savefig(savePath)
                  plt.show()
                  
              elif overrideSaveString == 'n':
                  print figNotSaved
                  
              else:
                  print inputError
                  print figNotSaved
        else:
             plt.savefig(savePath)
             plt.show()
             
    else:
        print figNotSaved
        
        
if __name__ == "__main__":
    
    # Generate some data for this demonstration.
    data = stats.norm.rvs(10.0, 2.5, size=500)
    
    # Fit a normal distribution to the data:
    mu, std = stats.norm.fit(data)
    
    # Plot the results
    plotDist(data, mu, std, num_bins=25, figNum=1,figName='fit_results',saveFig=False)
