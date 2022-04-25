import numpy as np
import matlab
import matlab.engine

from interaction_matrix import getIM
from jacobian import jacobianBody, adjointMatrix
from robots import Mat_M_camera, Mat_M_tool, Mat_S_body_camera, Mat_S_body_tool, Mat_S_space_camera, Mat_S_space_tool

RESOLUTION = np.array([2064, 2096])

QMIN = np.array([-np.pi, -2.41, -2.66, -np.pi, -2.23, -np.pi]) * 0.1
QMAX = np.array([np.pi, 2.41, 2.66, np.pi, 2.23, np.pi]) * 0.1

DQMIN = np.array([-1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218])
DQMAX = np.array([1.3963, 1.3963, 1.3963, 1.2218, 1.2218, 1.2218])

Ref_pt = np.array([1691.0, 302.0])
Ref_pg = np.array([854.0, 1681.0])
Ref_po = np.array([1587.0, 1417.0])

DT = 0.05  # 20hz

Ref_center = RESOLUTION / 2
# Ref_pg = Ref_center

T_ext = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1.0]])


def matlab_engine_launch():
    eng = matlab.engine.start_matlab()
    eng.cd("../Matlab/")
    print("Matlab current working directory is ---->>>", eng.pwd())
    return eng


# Launch matlab engine
eng = matlab_engine_launch()


# test interaction and jacobian matrices
# regulate to center
def feedbackLaw(qc: np.ndarray, dqc: np.ndarray, pg: np.ndarray, zg):
    # dp = IM @ Jcc @ dq
    # dq = pinv(IM@Jcc) @ dp
    # pinv(IM@Jcc) = pinv(Jcc) * pinv(IM)
    # dp = k*(Ref_p - p)
    # dq = pinv(Jcc) * pinv(IM) * k*(Ref_p - p)

    # dp = IM @ Vc
    # Vc = pinv(IM) * dp

    k = 0.1
    Jcc = jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)

    # P = getIM(pg, zg) @ J
    # dq = k * np.linalg.pinv(P) @ (Ref_center - pg)

    Vc = k * np.linalg.pinv(getIM(pg, zg)) @ (Ref_center - pg)
    # Vc = np.array([0,0,0,0,0,0.005])

    # T_ext = np.array([[0, 1, 0, 0],
    #                  [-1, 0, 0, 0],
    #                   [0, 0, 1, 0],
    #                   [0, 0, 0, 1.0]])
    #
    Vc = adjointMatrix(np.linalg.inv(T_ext)) @ Vc

    dq = np.linalg.pinv(Jcc) @ Vc

    return dq

# output dqc
def stateSs(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt, pg: np.ndarray, zg):
    Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)

    # Ref_Vc = np.array([0, 0, 0.02, 0, 0, 0])

    Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)  #

    Pcg_matlab = matlab.double(Pcg.tolist())
    Pct_matlab = matlab.double(Pct.tolist())
    qc_matlab = matlab.double(qc.tolist())
    dqc_matlab = matlab.double(dqc.tolist())
    pg_matlab = matlab.double(pg.tolist())
    pt_matlab = matlab.double(pt.tolist())

    dq = eng.optimization_Ss(Pcg_matlab, Pct_matlab, qc_matlab, dqc_matlab, pg_matlab, pt_matlab)
    return dq


def stateSa(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt, pg: np.ndarray, zg):
    Jcc = jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)
    Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ Jcc

    Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc

    Pcg_matlab = matlab.double(Pcg.tolist())
    Pct_matlab = matlab.double(Pct.tolist())
    Jcc_matlab = matlab.double(Jcc.tolist())
    qc_matlab = matlab.double(qc.tolist())
    dqc_matlab = matlab.double(dqc.tolist())
    pg_matlab = matlab.double(pg.tolist())
    pt_matlab = matlab.double(pt.tolist())

    # T = forwardKinematics(Mat_S_space_camera, Mat_M_camera, qc, 'space')
    # r = R.from_matrix(T[:3,:3])
    # r.as_rotvec()

    dq = eng.optimization_Sa(Pcg_matlab, Pct_matlab, Jcc_matlab, qc_matlab, dqc_matlab, pg_matlab, pt_matlab)
    return dq

def stateSc(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pg: np.ndarray, zg, ps: list):
    Jcc = jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)

    Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc

    Pcg_vertices = [(getIM(p, z) @ adjointMatrix(T_ext) @ Jcc).tolist() for p, z in ps]
    pgv = [p.tolist() for p, _ in ps]

    Pcg_matlab = matlab.double(Pcg.tolist())
    Jcc_matlab = matlab.double(Jcc.tolist())
    qc_matlab = matlab.double(qc.tolist())
    dqc_matlab = matlab.double(dqc.tolist())
    pg_matlab = matlab.double(pg.tolist())
    Pcg_vertices_matlab = matlab.double(Pcg_vertices)
    pgv_matlab = matlab.double(pgv)

    dq = eng.optimization_Sc(Pcg_matlab, Jcc_matlab, qc_matlab, dqc_matlab, pg_matlab, Pcg_vertices_matlab, pgv_matlab)
    return dq


# def stateSo(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pg: np.ndarray, zg, ps: list):
#     Jcc = jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)
#
#     Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc
#
#     Pcg_vertices = [(getIM(p, z) @ adjointMatrix(T_ext) @ Jcc).tolist() for p, z in ps]
#
#     Pcg_matlab = matlab.double(Pcg.tolist())
#     Jcc_matlab = matlab.double(Jcc.tolist())
#     qc_matlab = matlab.double(qc.tolist())
#     dqc_matlab = matlab.double(dqc.tolist())
#     pg_matlab = matlab.double(pg.tolist())
#     Pcg_vertices_matlab = matlab.double(Pcg_vertices)
#
#     dq = eng.optimization_Sa(Pcg_matlab, Jcc_matlab, qc_matlab, dqc_matlab, pg_matlab, Pcg_vertices_matlab)
#     return dq