import numpy as np
import matlab
import matlab.engine

import gurobipy as gp
from gurobipy import GRB

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

DT = 0.05  # 20hz

Ref_center = RESOLUTION / 2
Ref_pg = Ref_center

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


# env = gp.Env(empty=True)
# env.setParam("OutputFlag", 0)
# env.start()


# output dqc
def stateSs(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt, pg: np.ndarray, zg):
    # model = gp.Model('Ss')#, env=env

    # dq = model.addMVar(shape=6, ub=DQMAX, lb=DQMIN, obj=dqc, vtype=GRB.CONTINUOUS, name="dq")
    # dq.start = dqc
    # print(dqc)
    # print(qc)
    # model.update()
    # dq0 = model.addVar(lb=DQMIN[0], ub=DQMAX[0], name="dq0")
    # dq1 = model.addVar(lb=DQMIN[1], ub=DQMAX[1], name="dq1")
    # dq2 = model.addVar(lb=DQMIN[2], ub=DQMAX[2], name="dq2")
    # dq3 = model.addVar(lb=DQMIN[3], ub=DQMAX[3], name="dq3")
    # dq4 = model.addVar(lb=DQMIN[4], ub=DQMAX[4], name="dq4")
    # dq5 = model.addVar(lb=DQMIN[5], ub=DQMAX[5], name="dq5")

    # dpt = getIM(pt, zt) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc) @ dq
    Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)
    # pt_next = pt + dpt*DT

    # mat1 = getIM(pt, zt) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)

    # e1 = (mat1[0][0] * dq[0] + mat1[0][1] * dq[1] + mat1[0][2] * dq[2] + mat1[0][3] * dq[3] + mat1[0][4] * dq[4] +
    #       mat1[0][5] * dq[5]) * DT + pt[0]
    # e2 = (mat1[1][0] * dq[0] + mat1[1][1] * dq[1] + mat1[1][2] * dq[2] + mat1[1][3] * dq[3] + mat1[1][4] * dq[4] +
    #       mat1[1][5] * dq[5]) * DT + pt[1]
    #
    # e1 = (Ref_pt[0] ** 2 + e1 * e1 - 2 * Ref_pt[0] * e1)
    # e2 = (Ref_pt[1] ** 2 + e2- * e2 - 2 * Ref_pt[1] * e2)

    Ref_Vc = np.array([0, 0, 0.02, 0, 0, 0])
    #
    # mat2 = getIM(pg, zg) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)

    # e3 = (mat2[0][0] * dq[0] + mat2[0][1] * dq[1] + mat2[0][2] * dq[2] + mat2[0][3] * dq[3] + mat2[0][4] * dq[4] +
    #       mat2[0][5] * dq[5]) * DT + pg[0]
    # e4 = (mat2[1][0] * dq[0] + mat2[1][1] * dq[1] + mat2[1][2] * dq[2] + mat2[1][3] * dq[3] + mat2[1][4] * dq[4] +
    #       mat2[1][5] * dq[5]) * DT + pg[1]
    # pt_next = pt + dpt*DT
    # dpt = getIM(pt, zt) @ adjointMatrix(T_ext) @ Vc
    Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc)  #
    # Vc0 = mat3[0][0] * dq0 + mat3[0][1] * dq1 + mat3[0][2] * dq2 + mat3[0][3] * dq3 + mat3[0][4] * dq4 + mat3[0][5] * dq5
    # Vc1 = mat3[1][0] * dq0 + mat3[1][1] * dq1 + mat3[1][2] * dq2 + mat3[1][3] * dq3 + mat3[1][4] * dq4 + mat3[1][5] * dq5
    # Vc2 = mat3[2][0] * dq0 + mat3[2][1] * dq1 + mat3[2][2] * dq2 + mat3[2][3] * dq3 + mat3[2][4] * dq4 + mat3[2][5] * dq5
    # Vc3 = mat3[3][0] * dq0 + mat3[3][1] * dq1 + mat3[3][2] * dq2 + mat3[3][3] * dq3 + mat3[3][4] * dq4 + mat3[3][5] * dq5
    # Vc4 = mat3[4][0] * dq0 + mat3[4][1] * dq1 + mat3[4][2] * dq2 + mat3[4][3] * dq3 + mat3[4][4] * dq4 + mat3[4][5] * dq5
    # Vc5 = mat3[5][0] * dq0 + mat3[5][1] * dq1 + mat3[5][2] * dq2 + mat3[5][3] * dq3 + mat3[5][4] * dq4 + mat3[5][5] * dq5

    # e3 = (Ref_pg[0] - e3)**2#(Ref_pg[0] ** 2 + e3 * e3 - 2 * Ref_pg[0] * e3)
    # e4 = (Ref_pg[1] - e4)**2#(Ref_pg[1] ** 2 + e4 * e4 - 2 * Ref_pg[1] * e4)

    # e11 = (Ref_Vc[0] - Vc0) ** 2
    # e22 = (Ref_Vc[1] - Vc1) ** 2
    # e33 = (Ref_Vc[2] - Vc2) ** 2
    # e44 = (Ref_Vc[3] - Vc3) ** 2
    # e55 = (Ref_Vc[4] - Vc4) ** 2
    # e66 = (Ref_Vc[5] - Vc5) ** 2

    # dpg = getIM(pg, zg) @ adjointMatrix(T_ext) @ jacobianBody(Mat_S_space_camera, Mat_M_camera, qc) @ dq
    # pg_next = pg + dpg*DT

    # model.setObjective(e11 + e22 + e33 + e44 + e55 + e66, GRB.MINIMIZE)#0.5*e1 + 0.5*e2 +

    # model.addConstr(qc + dq * DT <= QMAX, name='qc_max')
    # model.addConstr(qc + dq * DT >= QMIN, name='qc_min')
    # model.addConstr(qc[1] + dq1 * DT <= QMAX[1])
    # model.addConstr(qc[2] + dq2 * DT <= QMAX[2])
    # model.addConstr(qc[4] + dq4 * DT <= QMAX[4])
    # model.addConstr(qc[1] + dq1 * DT >= QMIN[1])
    # model.addConstr(qc[2] + dq2 * DT >= QMIN[2])
    # model.addConstr(qc[4] + dq4 * DT >= QMIN[4])

    # model.addConstr(pg + dpg * DT <= RESOLUTION - np.array([1.0, 1.0]), name='g_in_fov_max')
    # model.addConstr(pg + dpg * DT >= np.array([0.0, 0.0]), name='g_in_fov_min')

    # model.Params.NonConvex = 2
    # model.optimize()

    # model.feasRelaxS(0, False, False, True)
    # model.optimize()
    # print(dq.X)

    # Vc0_ = mat3[0][0] * dq0.X + mat3[0][1] * dq1.X + mat3[0][2] * dq2.X + mat3[0][3] * dq3.X + mat3[0][4] * dq4.X + mat3[0][5] * dq5.X
    # Vc1_ = mat3[1][0] * dq0.X + mat3[1][1] * dq1.X + mat3[1][2] * dq2.X + mat3[1][3] * dq3.X + mat3[1][4] * dq4.X + mat3[1][5] * dq5.X
    # Vc2_ = mat3[2][0] * dq0.X + mat3[2][1] * dq1.X + mat3[2][2] * dq2.X + mat3[2][3] * dq3.X + mat3[2][4] * dq4.X + mat3[2][5] * dq5.X
    # Vc3_ = mat3[3][0] * dq0.X + mat3[3][1] * dq1.X + mat3[3][2] * dq2.X + mat3[3][3] * dq3.X + mat3[3][4] * dq4.X + mat3[3][5] * dq5.X
    # Vc4_ = mat3[4][0] * dq0.X + mat3[4][1] * dq1.X + mat3[4][2] * dq2.X + mat3[4][3] * dq3.X + mat3[4][4] * dq4.X + mat3[4][5] * dq5.X
    # Vc5_ = mat3[5][0] * dq0.X + mat3[5][1] * dq1.X + mat3[5][2] * dq2.X + mat3[5][3] * dq3.X + mat3[5][4] * dq4.X + mat3[5][5] * dq5.X

    # print('-->', np.array([Vc0_,Vc1_,Vc2_,Vc3_,Vc4_,Vc5_]))

    # status = model.Status
    # if status == GRB.UNBOUNDED:
    #     print('The model cannot be solved because it is unbounded')
    # if status == GRB.OPTIMAL:
    #     print('The optimal objective is %g' % model.ObjVal)
    # if status != GRB.INF_OR_UNBD and status != GRB.INFEASIBLE:
    #     print('Optimization was stopped with status %d' % status)
    # print('status:', status)

    # print('The model is infeasible; computing IIS')
    # model.computeIIS()
    # if model.IISMinimal:
    #     print('IIS is minimal\n')
    # else:
    #     print('IIS is not minimal\n')
    # print('\nThe following constraint(s) cannot be satisfied:')
    # for c in model.getConstrs():
    #     if c.IISConstr:
    #         print('%s' % c.ConstrName)
    # return np.array([dq0.X, dq1.X, dq2.X, dq3.X, dq4.X, dq5.X])

    Pcg_matlab = matlab.double(Pcg.tolist())
    Pct_matlab = matlab.double(Pct.tolist())
    # Ref_pg_matlab = matlab.double(Ref_pg.tolist())
    qc_matlab = matlab.double(qc.tolist())
    dqc_matlab = matlab.double(dqc.tolist())
    pg_matlab = matlab.double(pg.tolist())
    pt_matlab = matlab.double(pt.tolist())

    dq = eng.optimization_Ss(Pcg_matlab, Pct_matlab, qc_matlab, dqc_matlab, pg_matlab, pt_matlab)
    # dq = eng.test(1,2,3)
    print(dq)
    return dq


def stateSa(qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt, pg: np.ndarray, zg):
    pass
