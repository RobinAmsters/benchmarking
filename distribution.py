#!/usr/bin/env python2
# -*- coding: utf-8 -*-

""" 
1D data --> mu, std
OVERVIEW
"""

import numpy as np
import scipy.stats import norm
import matplotlib.pyplot as plt

def getDistribution(data):
	"""
	Standard python program to get normal dtbn from data.
	
	INPUT: filtered 1D data
	OUTPUT: mu, std
	"""
	mu, std = norm.fit(data)
	return mu,std
