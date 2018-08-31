import triad_openvr
import time
import sys
import csv
import signal
import numpy as np

from __future__ import print_function

outfile = "output/vive.csv"
poseOutfile = "output/pose.csv"
poseMatrix = list()

def writePositionToCsv(pose, csvWriter):
    csvWriter.writerow([str(time.time()), '%.4f' % (pose[0][3]*1000), '%.4f' % (pose[1][3]*1000), '%.4f' % (pose[2][3]*1000)])

def writePoseMatrices(sig, frame):
    np.save(poseOutfile, np.array(poseMatrix))
    exit(0)

signal.signal(signal.SIGINT, writePoseMatrices)

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

if len(sys.argv) == 1:
    interval = 1 / 50
elif len(sys.argv) == 2:
    interval = 1 / float(sys.argv[0])
else:
    print("Invalid number of arguments")
    interval = False
if interval:
    print(v.devices["tracker_1"].get_pose())
    with open(outfile, mode='w') as csvFile:
        csvWriter = csv.writer(csvFile)
        csvWriter.writerow(['timestamp', 'X', 'Y', 'Z'])
        while (True):
            start = time.time()
            txt = ""
            pose = v.devices["tracker_1"].get_pose()
            writePositionToCsv(pose, csvWriter)
            poseMatrix.append(np.array(pose))
            for i in range(3):
                txt += "%.4f" % pose[i][3]
                txt += " "
            print("\r" + txt, end="")
            sleep_time = interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)