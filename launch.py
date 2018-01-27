#!/usr/bin/env python2
# -*- coding: utf-8 -*-
""" created 27/1/2018
Python program that uses functions to benchmark sensors.

Phase 1: Try to use getFile functions successfully.
"""

import numpy as np
import scipy.io

from FileSelectGui import getFilePath
from bag import get_topic_data

testfile = getFilePath("example")
testdata = get_topic_data(testfile,"/NIS")

