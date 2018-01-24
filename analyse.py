#!/usr/bin/env python2
# -*- coding: utf-8 -*-

""" 
bagfile --> noise
OVERVIEW
"""

import
import
import

# Open bagfile -> filter -> position
bagFile = getFilePath('Select bag file').name
filtered_bagFile = filterBagFile(bagFile, "/scan")	# bagpath <-> bagfile? 
pose, all_t = get_joint_data(filtered_bagFile, 'base_footprint')

# (plot position)

cfr. bag.py

# make distribution

dbtn, mu, std = getDistribution(pose, all_t) #  (from distribution.py) for x
dbtn, mu, std = getDistribution(pose, all_t) #  (from distribution.py) for y

# plot distribution
plotDist(dbtn,mu,std) # for x
plotDist(dbtn,mu,std) # for y


