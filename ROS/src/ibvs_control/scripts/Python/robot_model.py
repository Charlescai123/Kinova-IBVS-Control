from typing import Tuple

import numpy as np
from math import *

# DH Parameter of Camera Arm Joint
DH_CAMERA_JOINT = np.array([[0, 0, 0, pi],
                            [0, -0.28481, 0, pi / 2],
                            [-pi / 2, -0.005375, 0.41, pi],
                            [-pi / 2, -0.006375, 0, pi / 2],
                            [pi, -0.31436, 0, pi / 2],
                            [pi, 0, 0, pi / 2]])

CAMERA_TO_END_EFFECTOR = np.array([pi / 2, -0.15593, 0.06, pi])

DH_CAMERA = np.vstack((DH_CAMERA_JOINT, CAMERA_TO_END_EFFECTOR))


class RobotModel:
    def __init__(self, jointDH: np.ndarray, DH: np.ndarray):
        self.jointDH = jointDH
        self.DH = DH
        self.Mat_S_space = get_twist(self.jointDH)  # Skew Matrix in Space Frame
        self.Mat_M = get_homeTrans(self.DH)  # Homogeneous Transform Matrix M


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


'''
Mat_S_space_camera = np.vstack((Sw, Sv))
Mat_M_camera = np.array([[0, 1, 0, 0],
                         [-1, 0, 0, 0.006375 - 0.005375 - 0.06],
                         [0, 0, 1, 0.15643 + 0.12838 + 0.41 + 0.20843 + 2 * 0.10593 + 0.05],
                         [0, 0, 0, 1]])
Mat_S_body_camera = jacobian.adjointMatrix(np.linalg.inv(Mat_M_camera)) @ Mat_S_space_camera

# ==== tool ====

Sw = np.array([[0.0, 0, -1],
               [0, 1, 0],
               [0, -1, 0],
               [0, 0, -1],
               [0, -1, 0],
               [0, 0, -1]]).T

P = np.array([[0, 0, 0],
              [0, 0, 0.12838 + 0.15643],
              [0, 0, 0.12838 + 0.15643 + 0.41],
              [0, 0.006375 - 0.005375, 0],
              [0, 0, 0.12838 + 0.15643 + 0.41 + 0.20843 + 0.10593],
              [0, 0.006375 - 0.005375, 0]]).T

P = np.array([[0, -0.5, 0],
              [0, 0, 0.12838 + 0.15643],
              [0, 0, 0.12838 + 0.15643 + 0.41],
              [0, 0.006375 - 0.005375 - 0.5, 0],
              [0, 0, 0.12838 + 0.15643 + 0.41 + 0.20843 + 0.10593],
              [0, 0.006375 - 0.005375 - 0.5, 0]]).T

Sv = np.zeros((3, 6))

for i in range(6):
    ss = jacobian.skewSymmetric(Sw[:, i])
    Sv[:, i] = -ss @ P[:, i]

Mat_S_space_tool = np.vstack((Sw, Sv))
Mat_M_tool = np.array([[-1, 0, 0, 0],
                       [0, -1, 0, 0.006375 - 0.005375],
                       [0, 0, 1, 0.15643 + 0.12838 + 0.41 + 0.20843 + 2 * 0.10593 + 0.0615 + 0.12],
                       [0, 0, 0, 1]])

Mat_M_tool = np.array([[-1, 0, 0, 0],
                       [0, -1, 0, 0.006375 - 0.005375 - 0.5],
                       [0, 0, 1, 0.15643 + 0.12838 + 0.41 + 0.20843 + 2 * 0.10593 + 0.0615 + 0.12],
                       [0, 0, 0, 1]])
Mat_S_body_tool = jacobian.adjointMatrix(np.linalg.inv(Mat_M_tool)) @ Mat_S_space_tool
'''
