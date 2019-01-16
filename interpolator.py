import numpy as np
import sys


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
        error += np.sum(np.abs(movedPositions[i, :] - np.dot(pose, np.append(originalPositions[i, :], 1))[:3]))
    if error > 30:
        return None
    return np.block([[R, meanPos], [0, 0, 0, 1]])  # TODO: Not sure about block


def overlap_datasets(markerTimePoints, transformedMarkerPositions, evaluationTrack, referenceFramerate=1/50):
    """

    :param initialMarkerPositions:
    :param markerTracks: The measured positions of the different markers, the measurements are assumed to be at a constant rate.
        The timestamps are assumed to be equal among the different markers.
    :param initialEvaluationPosition:
    :param evaluationTrack:
    :return:
    """
    evaluationTrack[:, 0] -= evaluationTrack[0, 0]
    kryptonIndices = get_virtual_indices(referenceFramerate, evaluationTrack[:, 0])
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


def get_transformed_vive_position(initialEvaluationPosition, initialVivePosition, viveMatrix):
    transformedVivePositions = list()
    for i in range(len(viveMatrix)):
        R = viveMatrix[i][0][:, :3]
        p = viveMatrix[i][0][:, 3] * 1000
        originP = p - np.dot(R, initialVivePosition) + initialVivePosition
        transformedVivePositions.append(originP + np.dot(R, initialEvaluationPosition))
    return transformedVivePositions


def timeShift(evaluationTrack, kryptonIndices, transformedMarkerPositions, markerTimePoints):
    """
    Find how much the evaluation track is shifted in time versus the reference (transformedMarkerPOsitions)
    :param evaluationTrack: numpy ndarray containing the positions of the system to evaluate: [[time, X, Y, Z]]
    :param kryptonIndices: a list of numbers containing the matching index in transformedMarkerPositions for each point in the evaluationtrack.
    :param transformedMarkerPositions: the positions of the measurement system to evaluate according to the Krypton system.
    :param markerTimePoints: the timestamps of transformedMarkerPositions
    :return:
    """
    minOverlap = 5  # seconds of overlap TODO: figure out realistic overlap
    minErr = sys.float_info.max

    finalOffset = 0
    finalR = None
    finalTranslation = None

    minOffset = - kryptonIndices[np.searchsorted(evaluationTrack[:, 0], evaluationTrack[-1, 0] - minOverlap)]
    maxOffset = np.searchsorted(markerTimePoints, markerTimePoints[-1] - minOverlap)
    offsets = range(minOffset, maxOffset)
    for offset in offsets:
        start = np.searchsorted(kryptonIndices + offset, 0, side='right')
        end = np.searchsorted(kryptonIndices + offset, len(transformedMarkerPositions))
        markerPoints = transformedMarkerPositions[kryptonIndices[start:end] + offset]
        evaluationPoints = evaluationTrack[start:end]

        evaluationTransformed, markerTransformed, R, translation = transform_to_common(evaluationPoints[:, 1:],
                                                                                       markerPoints)
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
    variance = np.sum((referenceSet - refMean) ** 2)
    if variance ** 0.5 < 2000:  # guarantee at least 5m of movement within interval.
        return sys.float_info.max
    return float(np.sum(np.abs(referenceSet - evaluationSet))) / len(referenceSet)


if __name__ == '__main__':
    c = np.array([[1, 2, 5], [3, 5, 4], [10, 8, 9]])
    m = np.array([[2, -1, 5], [5, -3, 4], [8, -10, 9]])
    pose = find_pose(c, m)
    print(pose)
