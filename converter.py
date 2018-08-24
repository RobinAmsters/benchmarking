import csv
import re
import krypton_static_points_selector

import numpy as np

from scipy.io import loadmat
from marvelmind import get_hedge_pos

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


def convert_file(filename):
    if isinstance(filename, str):
        if re.search('(.*)\.bag$', filename) is not None:
            convert_bag(filename)
        if re.search('(.*)\.mat$', filename) is not None:
            convert_mat(filename)


def convert_mat(filename):
    contents = loadmat(filename)
    datapoints = {k: v[:3, :].transpose() for k, v in contents.iteritems() if k.startswith('K6C_')}
    print(contents['Time'])
 #   referencePointNames = krypton_static_points_selector.create_steady_point_selector(datapoints)
 #    referencePoints = [datapoints[n] for n in referencePointNames]
    xVec, yVec, zVec = find_transformation_vectors(datapoints.values())
    # xVec = np.array([1,0,0])
    # yVec = np.array([0,1,0])
    # zVec = np.array([0,0,1])
    transFormedDataSets = list()
    for name, datapoint in datapoints.items():
        NaNvec = np.empty((1, datapoint.shape[1]))
        NaNvec[:] = np.nan
        datapoint[np.sum(datapoint, axis=1) == 0] = NaNvec
        X, Y, Z = transform_matrix(datapoint, xVec, yVec, zVec)
        transFormedDataSet = np.block([X, Y, Z])
        write_to_csv('/tmp/{}.csv'.format(name), [1./50 * i for i in range(len(transFormedDataSet))], transFormedDataSet)
    print(xVec)


def convert_bag(filename):
    positions, timestamps = get_hedge_pos(filename)
    write_to_csv('/tmp/hedge.csv', timestamps, positions * 1000)


def find_transformation_vectors(datasets):
    matrixList = np.ndarray((0, 3))
    for dataset in datasets:
        matrixList = np.block([[matrixList], [dataset]])
    tempMatrix = np.array(matrixList)
    del matrixList
    matrix = tempMatrix[abs(np.sum(tempMatrix, axis=1)) > 0]
    del tempMatrix
    meanVector = np.mean(matrix, axis=0)
    variance = matrix - meanVector
    (u, s, v) = np.linalg.svd(variance, full_matrices=False)
    xVec = v[0]
    yVec = v[1]
    zVec = v[2]
    return xVec, yVec, zVec


def transform_matrix(matrix, xVec, yVec, zVec):
    meanVector = np.nanmean(matrix, axis=0)
    variance = matrix # - meanVector
    xVec.shape = (1, 3)
    yVec.shape = (1, 3)
    zVec.shape = (1, 3)
    X = np.dot(variance, xVec.transpose())
    Y = np.dot(variance, yVec.transpose())
    Z = np.dot(variance, zVec.transpose())
    return X, Y, Z


def write_to_csv(fileName, timestamps, positions):
    with open(fileName, 'w') as outputFile:
        writer = csv.writer(outputFile)
        header = ['timestamp', 'X', 'Y', 'Z']
        writer.writerow(header)
        for i in range(positions.shape[0]):
            writer.writerow([timestamps[i]] + list(positions[i]))


if __name__ == '__main__':
    convert_file('/home/quinten/Documents/vakantiejob/results/Video/2016-02-11-11-42-57.bag')