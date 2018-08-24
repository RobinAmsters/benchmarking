import numpy as np
import csv
import sys
import interpolator
import split_datasets

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# The name of the krypton markers and their position from the middle in mm
staticMarkerPositions = {'K6C_12250_3_1': np.array((-110, -110, 0)), 'K6C_12250_3_2': np.array((-110, 110, 0)),
                         'K6C_12250_3_3': np.array((110, 110, 0)), 'K6C_12250_3_4': np.array((110, -110, 0)),
                         'K6C_12250_3_5': np.array((0, 0, 0)), 'K6C_12250_3_6': np.array((-60, -60, 0))}


def csvReader(filename):
    with open(filename, mode='rb') as input:
        reader = csv.reader(input)
        headerRow = reader.next()
        if headerRow == ['timestamp', 'X', 'Y', 'Z']:
            return np.array([[float(elem) for elem in row] for row in reader])


def plot_results(markerTrack, evaluationTrack):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(markerTrack[:, 0], markerTrack[:, 1], markerTrack[:, 2], c='b')
    ax.scatter(evaluationTrack[:, 0], evaluationTrack[:, 1], evaluationTrack[:, 2], c='r')
    plt.show()

def error_func(markerTrack, evaluationTrack):
    error = []
    iterLen = min(len(markerTrack), len(evaluationTrack))
    for i in range(iterLen):
        if not np.isnan(np.sum(markerTrack[i])) and not np.isnan(np.sum(evaluationTrack[i])):
            error.append(np.sqrt(np.sum((markerTrack[i] - evaluationTrack[i])**2)))
    return error

def sync_timestamps(markerTimestamps, evaluationTimestamps, offset):
    newEvaluationtimestamps = evaluationTimestamps - evaluationTimestamps[0]
    if offset > 0:
        newEvaluationtimestamps += markerTimestamps[offset]
    elif abs(offset) < len(markerTimestamps):
        newEvaluationtimestamps += markerTimestamps[0] - markerTimestamps[-offset]
    else:
        frequency = float(markerTimestamps[-1] - markerTimestamps[0]) / (len(markerTimestamps) - 1)
        newEvaluationtimestamps += offset / frequency
    return newEvaluationtimestamps


if __name__ == '__main__':
    kryptonPoints = ['K6C_12250_3_{}.csv'.format(i) for i in range(1, 7)]
    kryptonFolder = '/home/quinten/Documents/vakantiejob/results/All/'
    kryptonPositions = staticMarkerPositions
    benchmarkFile = '/home/quinten/Documents/vakantiejob/results/All/hedge.csv'
    marvelmindPos = np.array((-60, 60, 0))
    pozyxPos = np.array((80,-55,0))
    camerapos = np.array((55,52,0))
    benchmarkPos = marvelmindPos

    markerPositions = []
    markerTracks = []
    evaluationTrack = csvReader(benchmarkFile)
    leastNan = 0
    nanAmount = sys.maxint

    for i in range(len(kryptonPoints)):
        kryptonPoint = kryptonPoints[i]
        track = csvReader(kryptonFolder + kryptonPoint)
        position = staticMarkerPositions[kryptonPoint[:-4]]
        markerTracks.append(track)
        markerPositions.append(position)
        if sum(np.isnan(track[:, 1])) < nanAmount:
            leastNan = i

    markerTracks = np.array(markerTracks)

    splitPoint = split_datasets.create_steady_point_selector(markerTracks[leastNan, :, 1:])

    transformedMarkerPositions = interpolator.get_transformed_marker_position(benchmarkPos, markerPositions,
                                                                              markerTracks)
    markerTimePoints = markerTracks[0, :, 0]

    offset, R, translation = interpolator.overlap_datasets(markerTimePoints,
                                                           transformedMarkerPositions[:splitPoint],
                                                           evaluationTrack)

    print offset

    evalTime = sync_timestamps(markerTimePoints, evaluationTrack[:,0], offset)
    splitPointTime = markerTimePoints[splitPoint]

    evalMarkers = transformedMarkerPositions[splitPoint:]
    evalSet = np.dot(R, evaluationTrack[evalTime > splitPointTime, 1:].transpose()).transpose() + translation

    markerIndices = interpolator.get_virtual_indices(50, evalTime[evalTime > splitPointTime] - splitPointTime)

    errorDistribution = error_func(evalMarkers[markerIndices], evalSet)
    print("Error:\nMean: {}\nP95: {}".format(np.mean(errorDistribution), np.percentile(errorDistribution, 95)))

    plot_results(evalMarkers[markerIndices], evalSet)

    # print evalTime
    print(errorDistribution)

