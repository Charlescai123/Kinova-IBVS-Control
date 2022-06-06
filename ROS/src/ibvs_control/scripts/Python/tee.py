from copy import copy, deepcopy
from typing import Tuple

import numpy
import robot_model as model
import file_reader as reader
import numpy as np
from math import *

np.set_printoptions(suppress=True, threshold=np.inf)


def skewSymmetric(p: np.ndarray):
    assert p.shape[0] == 3
    return np.array([[0, -p[2], p[1]],
                     [p[2], 0, -p[0]],
                     [-p[1], p[0], 0]])


def get_transMatrix(DH: np.array) -> np.array:
    assert len(DH) == 4
    theta, d, a, alpha = DH
    T = np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                  [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                  [0, sin(alpha), cos(alpha), d],
                  [0, 0, 0, 1]])
    return T


def get_twist(DH: np.ndarray) -> np.ndarray:
    nRows, nCols = DH.shape
    assert nCols == 4
    Sw = np.zeros((3, nRows), dtype=float)
    Sv = np.zeros((3, nRows), dtype=float)
    P = np.zeros((3, nRows), dtype=float)
    T = np.eye(4)
    for i in range(nRows):
        T = T @ get_transMatrix(DH[i])
        Sw[:, i] = np.trunc(T[0:3, 2])  # Get Z axis
        P_O = T[0:3, 3]
        P[:, i] = np.logical_not(Sw[:, i]) * P_O  # Perpendicular distance to Z axis
        ss = skewSymmetric(Sw[:, i])
        Sv[:, i] = -ss @ P[:, i]
    return np.vstack((Sw, Sv))


def get_homeTrans(DHs: np.ndarray) -> np.ndarray:
    nRows, nCols = DHs.shape
    assert nCols == 4
    HT = np.eye(4)
    for i in range(nRows):
        HT = HT @ get_transMatrix(DHs[i])
    return HT


DH_Param = np.array([[0, 0, 0, pi],
                     [0, -0.28481, 0, pi / 2],
                     [-pi / 2, -0.005375, 0.41, pi],
                     [-pi / 2, -0.006375, 0, pi / 2],
                     [pi, -0.31436, 0, pi / 2],
                     [pi, 0, 0, pi / 2]])

DH_Param_All = np.array([[0, 0, 0, pi],
                         [0, -0.28481, 0, pi / 2],
                         [-pi / 2, -0.005375, 0.41, pi],
                         [-pi / 2, -0.006375, 0, pi / 2],
                         [pi, -0.31436, 0, pi / 2],
                         [pi, 0, 0, pi / 2],
                         [pi / 2, -0.15593, 0.06, pi]])

S = get_twist(DH_Param)
M = get_homeTrans(DH_Param_All)

print(S)
print(M)
a = np.array([])
print(a)

pos_limit, vel_limit = reader.joint_limit_load()
print(type(pos_limit))
print(pos_limit)
print(vel_limit)

Q_MIN = np.array([-np.pi, -2.41, -2.66, -np.pi, -2.23, -np.pi])
Q_MAX = np.array([np.pi, 2.41, 2.66, np.pi, 2.23, np.pi])
Q_LIMIT = np.vstack((Q_MIN, Q_MAX))
print(type(Q_LIMIT))
print(Q_LIMIT)
