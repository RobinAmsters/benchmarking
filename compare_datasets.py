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
                         'K6C_12250_3_5': np.array((0, 0, 0)), 'K6C_12250_3_6': np.array((-60, -58.65, 0))}


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
    """
    Transform the timestamps of the evaluation data to the time-domain of the marker data.
    :param markerTimestamps:
    :param evaluationTimestamps:
    :param offset:
    :return: The new timestamps for the evaluation data.
    """
    newEvaluationtimestamps = evaluationTimestamps - evaluationTimestamps[0]
    if offset >= 0:
        newEvaluationtimestamps += markerTimestamps[offset]
    elif abs(offset) < len(markerTimestamps):
        newEvaluationtimestamps += markerTimestamps[0] + (markerTimestamps[0] - markerTimestamps[abs(offset)])
    else:
        frequency = float(markerTimestamps[-1] - markerTimestamps[0]) / (len(markerTimestamps) - 1)
        newEvaluationtimestamps += offset / frequency + markerTimestamps[0]
    return newEvaluationtimestamps


def get_krypton_track(kryptonFolder, kryptonPoints, benchmarkPos):
    markerTracks = list()
    markerPositions = list()
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

    return transformedMarkerPositions, markerTimePoints, splitPoint

def get_vive_track(vivePoseFile, vivePos, benchmarkPos):
    viveMatrix = np.load(vivePoseFile)
    for i in range(len(viveMatrix)):
        if np.sum(viveMatrix[i][0][:, 3]) == 0:
            viveMatrix[i][0][:, 3] = np.array((np.nan, np.nan, np.nan))
    newPoints = interpolator.get_transformed_vive_position(benchmarkPos, vivePos, viveMatrix)
    markerTrack = np.array(newPoints)
    splitPoint = split_datasets.create_steady_point_selector(markerTrack)
    return markerTrack, np.array(range(len(markerTrack))) * 0.020078839783579813, splitPoint


if __name__ == '__main__':

    # Parameters
    viveReference = False
    kryptonPoints = ['K6C_12250_3_{}.csv'.format(i) for i in range(1, 7)] # niet nodig wanneer viveReference = True
    kryptonFolder = '/home/quinten/Documents/vakantiejob/results/changed_kryoton_pos/' # niet nodig wanneer viveReference = True
    kryptonPositions = staticMarkerPositions # niet nodig wanneer viveReference = True
    benchmarkFile = '/home/quinten/Documents/vakantiejob/results/changed_kryoton_pos/hedge.csv'
    viveFile = '/home/quinten/Documents/vakantiejob/results/temp/t1/pose.csv.npy' # niet nodig wanneer viveReference = False
    marvelmindPos = np.array((-60, 60, 0))
    pozyxPos = np.array((80,-55,0))
    camerapos = np.array((55,52,0))
    vivepos = np.array((54.75, 49.75, 0))
    benchmarkPos = marvelmindPos
    referenceFramerate = 1/0.02 # 1/0.020078839783579813 voor Vive

    # Load the track to evaluate
    evaluationTrack = csvReader(benchmarkFile)

    # Get the equivalent positions of the evaluation system based on the reference system
    if viveReference:
        transformedMarkerPositions, markerTimePoints, splitPoint = get_vive_track(viveFile, vivepos, benchmarkPos)
        plot_results(transformedMarkerPositions, evaluationTrack)
    else:
        transformedMarkerPositions, markerTimePoints, splitPoint = get_krypton_track(kryptonFolder, kryptonPoints, benchmarkPos)

    # Fit both datasets in time, rotation and translation
    offset, R, translation = interpolator.overlap_datasets(markerTimePoints[:splitPoint],
                                                           transformedMarkerPositions[:splitPoint],
                                                           evaluationTrack,
                                                           referenceFramerate)

    # Set the timestamps of the evaluation set based on the offset previously obtained
    # (the timestamps of the reference system and the evaluation system shoud be synced after this).
    evalTime = sync_timestamps(markerTimePoints, evaluationTrack[:,0], offset)

    # Get the timestamp of the splitpoint
    # (splitpoint determines which part of the reference dataset should be used for fitting and which part shoud be used for evaluation)
    splitPointTime = markerTimePoints[splitPoint]

    # Create the evaluation datasets based on the transormations and timeshifts
    evalMarkers = transformedMarkerPositions[splitPoint:]
    evalSet = np.dot(R, evaluationTrack[evalTime >= splitPointTime, 1:].transpose()).transpose() + translation

    markerIndices = interpolator.get_virtual_indices(referenceFramerate, evalTime[evalTime >= splitPointTime] - splitPointTime)
    markerIndices = markerIndices[markerIndices < len(evalMarkers)]

    # Calculate the distance between the reference and the evaluation dataset at each point.
    errorDistribution = error_func(evalMarkers[markerIndices], evalSet)
    print("Error:\nMean: {}\nMedian: {}\nP95: {}".format(np.mean(errorDistribution), np.median(errorDistribution), np.percentile(errorDistribution, 95)))

    plt.close()
    evalTrack = evalMarkers[markerIndices]
    filter = ~np.isnan(np.sum(evalTrack, axis=1))

    # Plot the results of the evaluation
    plot_results(evalMarkers[markerIndices], evalSet[:len(evalTrack), :][filter])


    # Plot full synced tracks
    plot_results(transformedMarkerPositions, np.dot(R, evaluationTrack[:, 1:].transpose()).transpose() + translation)

