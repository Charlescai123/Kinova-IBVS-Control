import numpy as np


def skewSymmetric(p: np.ndarray):
    if p.shape[0] != 3:
        print("p.shape !=3 !!!!!!!!!!!!!!!!!")
    return np.array([[0, -p[2], p[1]],
                     [p[2], 0, -p[0]],
                     [-p[1], p[0], 0]])


def adjointMatrix(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    p_ss = skewSymmetric(p)
    Adj = np.vstack(
        (
            np.hstack(
                (R, np.zeros((3, 3)))
            ),
            np.hstack((p_ss @ R, R))
        )
    )

    return Adj


def angle2Rot(axis: np.ndarray, theta):
    if np.linalg.norm(axis) == 0.0:
        return np.eye(3)
    else:
        ss = skewSymmetric(axis)
        R = np.eye(3) + np.sin(theta) * ss + (1 - np.cos(theta)) * (ss @ ss)
        return R


def twist2Trans(S: np.ndarray, theta):
    # print(S)

    omega = S[:3]
    # print(omega)
    R = angle2Rot(omega, theta)
    # print(R)

    v = S[3:]
    ss = skewSymmetric(omega)
    # print(v)
    # print(ss)
    t = (np.eye(3) * theta + (1 - np.cos(theta)) * ss + (theta - np.sin(theta)) * (ss @ ss)) @ v
    # print(t)
    # print(t.reshape(3,1))

    T = np.vstack((np.hstack((R, t.reshape(3, 1))), np.array([0, 0, 0, 1])))
    return T


def forwardKinematics(M_S: np.ndarray, M, q, frame='space'):
    nrows, ncols = M_S.shape
    T = np.eye(4)

    for i in range(ncols):
        T = T @ twist2Trans(M_S[:, i], q[i])
    if frame == 'space':
        T = T @ M
    elif frame == 'body':
        T = M @ T

    return T


def jacobian(M_S: np.ndarray, q):
    nrows, ncols = M_S.shape
    J = np.zeros((6, ncols))
    T = np.eye(4)
    for i in range(ncols):
        T = T @ twist2Trans(M_S[:, i], q[i])
        J[:, i] = adjointMatrix(T) @ M_S[:, i]

    return J


def jacobianBody(M_S: np.ndarray, M, q):
    J_s = jacobian(M_S, q)
    T = forwardKinematics(M_S, M, q, 'space')
    J_b = adjointMatrix(np.linalg.inv(T)) @ J_s
    return J_b
