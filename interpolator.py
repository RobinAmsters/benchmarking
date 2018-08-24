import numpy as np
import sys
import itertools

from numpy.core.multiarray import ndarray


def interpolate_based_on_dataset(dataset, timepoint):
    """
    Linear interpolation to find the coordinate values from the dataset at the given timepoint.

    :param dataset: np.ndarray with rows containing [timestamp, X, Y, Z]
    :param timepoint: timestamp to calculate position of
    :return: np.ndarray([X, Y, Z])
    """
    lowerIndex = np.searchsorted(dataset[:, 0], timepoint)
    higherIndex = lowerIndex + 1
    if higherIndex == len(dataset[:, 0]):
        raise Exception(
            "The datarange of the dataset is to small to ifnd the timepoint, this function cannot extrapolate.")
    factor = float(timepoint - dataset[lowerIndex, 0]) / (dataset[higherIndex, 0] - dataset[lowerIndex, 0])
    return dataset[lowerIndex, 1:] + factor * (dataset[higherIndex, 1:] - dataset[lowerIndex, 1:])


def transform_to_common(dataset1, dataset2):
    """
    Transform both datasets to a common coordinate system, assume both datasets have the same amount of points.
    :param dataset1:
    :param dataset2:
    :return:
    """
    keep = np.invert(np.bitwise_or(np.isnan(np.sum(dataset1, axis=1)), np.isnan(np.sum(dataset2, axis=1))))
    dataset1 = dataset1[keep]
    dataset2 = dataset2[keep]
    mean1 = np.mean(dataset1, axis=0)
    mean2 = np.mean(dataset2, axis=0)
    zeroAvg1 = dataset1 - mean1
    zeroAvg2 = dataset2 - mean2
    R = extract_rotation_matrix(zeroAvg1, zeroAvg2)
    tranformed1 = np.dot(R, zeroAvg1.transpose()).transpose()
    translation = np.mean(dataset2 - np.dot(R, dataset1.transpose()).transpose(), axis=0)
    return tranformed1, zeroAvg2, R, translation


def extract_rotation_matrix(zeroAveragedPositions1, zeroAveragedPositions2):
    """
    Find the rotation matrix between two point clouds with a mean position of 0. The rotatation matrix is defined as follows:
    R . zeroAveragedPositions1 == zeroAveragedPositions2 + epsilon
    :param zeroAveragedPositions1: The first point cloud
    :param zeroAveragedPositions2: The second point cloud
    :return: The rotation matrix
    """
    u, s, v_trans = np.linalg.svd(np.dot(zeroAveragedPositions1.transpose(), zeroAveragedPositions2))
    d = np.linalg.det(np.dot(v_trans.transpose(), u.transpose()))
    R = np.dot(np.dot(v_trans.transpose(), np.array([[1, 0, 0], [0, 1, 0], [0, 0, d]])), u.transpose())
    return R


def find_pose(originalPositions, movedPositions):
    """
    Find the pose matrix for a given set of marker positions on the robot and the coordinates of these points for which to calculate the pose matrix.
    :param originalPositions: The positions of the markers on the robot (rows are different markers).
    :param movedPositions: The moved positions (rows are different markers).
    :return: The pose matrix.
    """
    if len(originalPositions) != len(movedPositions):
        raise Exception(
            "To find the pose of the robot, the same amount of markers have to be defined as the amount of measured markers")
    indices = np.invert(np.isnan(np.sum(movedPositions, axis=1)))
    if sum(indices) < 3:
        return None
    originalPositions = np.array(originalPositions)[indices, :]
    movedPositions = movedPositions[indices, :]
    zeroAvg1 = originalPositions - np.mean(originalPositions, axis=0)
    zeroAvg2 = movedPositions - np.mean(movedPositions, axis=0)
    R = extract_rotation_matrix(zeroAvg1, zeroAvg2)
    transformed_input = np.dot(R, originalPositions.transpose()).transpose()
    meanVec = np.mean(transformed_input, axis=0)
    meanPos = np.mean(movedPositions, axis=0) - meanVec
    meanPos.shape = (3, 1)
    pose = np.block([[R, meanPos], [0, 0, 0, 1]])
    error = 0
    for i in range(originalPositions.shape[0]):
        error += np.sum(np.abs(movedPositions[i, :] - np.dot(pose, np.append(originalPositions[i,:],1))[:3]))
    if error > 30:
        return None
    return np.block([[R, meanPos], [0, 0, 0, 1]])  # TODO: Not sure about block


def match_rate(referenceTimeStamps, measuredPoints):
    rate = len(referenceTimeStamps) / float(referenceTimeStamps[-1] - referenceTimeStamps[0])
    measuredPoints[:, 0] -= measuredPoints[0, 0]
    newPoints = list()
    i = 0
    while i * rate < measuredPoints[-1, 0]:
        newPoints.append([rate * i] + interpolate_based_on_dataset(measuredPoints, rate * i))
        i += 1
    return np.array(newPoints), rate


def match_datasets(initialMarkerPositions, markerTracks, initialEvaluationPosition, evaluationTrack):
    """

    :param initialMarkerPositions:
    :param markerTracks: The measured positions of the different markers, the measurements are assumed to be at a constant rate.
        The timestamps are assumed to be equal among the different markers.
    :param initialEvaluationPosition:
    :param evaluationTrack:
    :return:
    """
    evaluationTrackSynced, rate = match_rate(markerTracks[0, 1:, 0], evaluationTrack)
    minOverlap = int(rate * 4)  # TODO: figure out realistic overlap

    transformedEvaluationPositions = list()

    for i in range(markerTracks.shape[1]):
        pose = find_pose(initialMarkerPositions, markerTracks[:, i, 1:])
        transformedEvaluationPositions.append(np.dot(pose, initialEvaluationPosition))
    transformedEvaluationPositions = np.array(transformedEvaluationPositions)

    transformedLen = len(transformedEvaluationPositions)
    evaluationSyncLen = len(evaluationTrackSynced)
    nb_iterations = transformedLen + evaluationSyncLen - 2 * minOverlap

    minErr = sys.float_info.max
    markerStart = 0
    evaluationStart = 0

    for i in range(nb_iterations):
        if transformedLen - i - minOverlap > minOverlap:
            transFStart = transformedLen - i - minOverlap
            evalTStart = 0
        else:
            transFStart = minOverlap
            evalTStart = i + 2 * minOverlap - transformedLen
        # transFStart = 0
        # evalTStart = 0
        overlapLen = min(transformedLen - transFStart, evaluationSyncLen - evalTStart)

        originalTransformed, measuredTransformed = transform_to_common(
            transformedEvaluationPositions[transFStart:transFStart + overlapLen, :],
            evaluationTrackSynced[evalTStart:evalTStart + overlapLen, :])
        error = error_func(originalTransformed, measuredTransformed)
        if error < minErr:
            minErr = error
            markerStart = transFStart
            evaluationStart = evalTStart

    return markerStart, evaluationStart


def overlap_datasets(markerTimePoints, transformedMarkerPositions, evaluationTrack):
    """

    :param initialMarkerPositions:
    :param markerTracks: The measured positions of the different markers, the measurements are assumed to be at a constant rate.
        The timestamps are assumed to be equal among the different markers.
    :param initialEvaluationPosition:
    :param evaluationTrack:
    :return:
    """
    kryptonRate = 50  # markerTracks.shape[1]/(markerTracks[0, -1, 0] - markerTracks[0, 0,0])
    evaluationTrack[:, 0] -= evaluationTrack[0, 0]
    kryptonIndices = get_virtual_indices(kryptonRate, evaluationTrack[:, 0])
    return timeShift(evaluationTrack, kryptonIndices, transformedMarkerPositions, markerTimePoints)


def get_transformed_marker_position(initialEvaluationPosition, initialMarkerPositions, markerTracks):
    transformedMarkerPositions = list()
    for i in range(markerTracks.shape[1]):
        pose = find_pose(initialMarkerPositions, markerTracks[:, i, 1:])
        if pose is not None:
            transformedMarkerPositions.append(np.dot(pose, np.append(initialEvaluationPosition, 1))[:3])
        else:
            transformedMarkerPositions.append(np.array([np.nan, np.nan, np.nan]))
    transformedMarkerPositions = np.array(transformedMarkerPositions)
    return transformedMarkerPositions


def timeShift(evaluationTrack, kryptonIndices, transformedMarkerPositions, markerTimePoints):
    minOverlap = 15  # seconds of overlap TODO: figure out realistic overlap
    minErr = sys.float_info.max

    finalOffset = 0
    finalR = None
    finalTranslation = None

    minOffset = - np.searchsorted(evaluationTrack[:, 0], evaluationTrack[-1, 0] - minOverlap)
    maxOffset = np.searchsorted(markerTimePoints, markerTimePoints[-1] - minOverlap)
    offsets = range(minOffset, maxOffset)
    for offset in offsets:
        start = max(0, -offset)
        end = np.searchsorted(kryptonIndices + offset, len(transformedMarkerPositions))
        markerPoints = transformedMarkerPositions[kryptonIndices[start:end] + offset]
        evaluationPoints = evaluationTrack[start:end]

        evaluationTransformed, markerTransformed, R, translation = transform_to_common(evaluationPoints[:, 1:], markerPoints)
        error = error_func(markerTransformed, evaluationTransformed)

        if error < minErr:
            minErr = error
            finalOffset = offset
            finalR = R
            finalTranslation = translation

    return finalOffset, finalR, finalTranslation


def get_virtual_indices(kryptonRate, timestamps):
    return np.array(map(lambda a: int(round(a * kryptonRate)), timestamps))


def error_func(referenceSet, evaluationSet):
    refMean = np.mean(referenceSet, axis=0)
    variance = np.sum((referenceSet - refMean)**2)
    if variance**0.5 < 5000:
        return sys.float_info.max
    return float(np.sum(np.abs(referenceSet - evaluationSet))) / len(referenceSet)


def find_overlapping():
    pass

if __name__ == '__main__':
    c = np.array([[1, 2, 5], [3, 5, 4], [10, 8, 9]])
    m = np.array([[2, -1, 5], [5, -3, 4], [8, -10, 9]])
    pose = find_pose(c, m)
    print(pose)