from typing import Tuple

import numpy as np
import file_reader as reader
from math import *

'''
DH Parameter of Camera Arm Joint
'''
DH_CAMERA, DH_CAMERA_JOINT, JOINT_TO_END_EFFECTOR = reader.DH_load()

'''
Camera Joint Position/Velocity limit
'''
Q_LIMIT, DQ_LIMIT = reader.joint_limit_load()


class RobotModel:
    def __init__(self, _DH: np.ndarray, _jointDH: np.ndarray):
        self.DH = _DH
        self.jointDH = _jointDH
        self.Mat_M = get_homeTrans(self.DH)  # Homogeneous Transform Matrix M
        self.Mat_S_space = get_twist(self.jointDH)  # Skew Matrix in Space Frame


'''Robot Model Calc Function
'''


def skewSymmetric(p: np.ndarray) -> np.ndarray:
    assert p.shape[0] == 3
    return np.array([[0, -p[2], p[1]],
                     [p[2], 0, -p[0]],
                     [-p[1], p[0], 0]])


def get_transMatrix(DH: np.ndarray) -> np.ndarray:
    assert len(DH) == 4
    theta, d, a, alpha = DH
    T = np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                  [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                  [0, sin(alpha), cos(alpha), d],
                  [0, 0, 0, 1]])
    return T


def get_twist(DHs: np.ndarray) -> np.ndarray:
    nRows, nCols = DHs.shape
    assert nCols == 4
    Sw = np.zeros((3, nRows), dtype=float)
    Sv = np.zeros((3, nRows), dtype=float)
    P = np.zeros((3, nRows), dtype=float)
    T = np.eye(4)
    for i in range(nRows):
        T = T @ get_transMatrix(DHs[i])
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


def adjointMatrix(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    p_ss = skewSymmetric(p)
    Adj = np.vstack(
        (np.hstack((R, np.zeros((3, 3)))), np.hstack((p_ss @ R, R)))
    )
    return Adj


def angle2Rot(axis: np.ndarray, theta):
    if np.linalg.norm(axis) == 0.0:
        return np.eye(3)
    else:
        ss = skewSymmetric(axis)
        R = np.eye(3) + np.sin(theta) * ss + (1 - np.cos(theta)) * (ss @ ss)
        return R


def twist2Trans(S: np.ndarray, theta) -> np.ndarray:
    w = S[:3]
    R = angle2Rot(w, theta)
    v = S[3:]
    ss = skewSymmetric(w)
    t = (np.eye(3) * theta + (1 - np.cos(theta)) * ss + (theta - np.sin(theta)) * (ss @ ss)) @ v
    T = np.vstack((np.hstack((R, t.reshape(3, 1))), np.array([0, 0, 0, 1])))
    return T


def forward_kinematics(M_S: np.ndarray, M, q, frame='space'):
    nRows, nCols = M_S.shape
    T = np.eye(4)
    for i in range(nCols):
        T = T @ twist2Trans(M_S[:, i], q[i])
    if frame == 'space':
        T = T @ M
    elif frame == 'body':
        T = M @ T
    return T


def jacobian_space(M_S: np.ndarray, q):
    nRows, nCols = M_S.shape
    J = np.zeros((6, nCols))
    T = np.eye(4)
    for i in range(nCols):
        T = T @ twist2Trans(M_S[:, i], q[i])
        J[:, i] = adjointMatrix(T) @ M_S[:, i]
    return J


def jacobian_body(M_S: np.ndarray, M, q):
    J_s = jacobian_space(M_S, q)
    T = forward_kinematics(M_S, M, q, 'space')
    J_b = adjointMatrix(np.linalg.inv(T)) @ J_s
    return J_b
