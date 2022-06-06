import numpy as np
import matlab
import matlab.engine

from interaction_matrix import getIM
from robot_model import *

# Ref_pt = np.array([520, 110])  # TCP Reference Point
# Ref_pg = np.array([180, 396])  # Goal Reference Point
# Ref_po = np.array([417, 309])  # Obstacle Reference Point
# Ref_center = RESOLUTION / 2  # Image Center Reference Point


# DT = 0.05  # 20hz
#
T_ext = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])


def matlab_engine_launch():
    eng = matlab.engine.start_matlab()
    eng.cd("../Matlab/")
    print("Matlab current working directory is ---->>>", eng.pwd())
    return eng


'''
Optimizer for state machine
'''

np.set_printoptions(suppress=True, threshold=np.inf)


class StateMachineOptimizer:
    def __init__(self, _Ref_pt: np.ndarray, _Ref_pg: np.ndarray, _Ref_po: np.ndarray, _Resolution: np.ndarray,
                 _DH_Camera: np.ndarray, _DH_Camera_Joint: np.ndarray, Q_Limit: np.ndarray, DQ_Limit: np.ndarray):
        self.q_lim = Q_Limit  # Q_Limit
        self.dq_lim = DQ_Limit  # DQ_Limit
        self.ref_pt = _Ref_pt  # TCP Reference Point
        self.ref_pg = _Ref_pg  # Goal Reference Point
        self.ref_po = _Ref_po  # Obstacle Reference Point
        self.resolution = _Resolution  # Resolution
        self.eng = matlab_engine_launch()  # Launch matlab engine
        self.camArm = RobotModel(_DH_Camera, _DH_Camera_Joint)  # Camera Arm Model
        # print(self.camArm.Mat_S_space)
        # print(self.camArm.Mat_M)

    # test interaction and jacobian matrices, regulate to center
    def feedbackLaw(self, qc: np.ndarray, dqc: np.ndarray, pg: np.ndarray, zg):
        # dp = IM @ Jcc @ dq
        # dq = pinv(IM@Jcc) @ dp
        # pinv(IM@Jcc) = pinv(Jcc) * pinv(IM)
        # dp = k*(Ref_p - p)
        # dq = pinv(Jcc) * pinv(IM) * k*(Ref_p - p)

        # dp = IM @ Vc
        # Vc = pinv(IM) * dp

        k = 0.1
        Jcc = jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)

        # P = getIM(pg, zg) @ J
        # dq = k * np.linalg.pinv(P) @ (Ref_center - pg)

        Ref_center = self.resolution / 2  # Image Center Reference Point
        Vc = k * np.linalg.pinv(getIM(pg, zg)) @ (Ref_center - pg)

        # Vc = np.array([0,0,0,0,0,0.005])

        Vc = adjointMatrix(np.linalg.inv(T_ext)) @ Vc
        dq = np.linalg.pinv(Jcc) @ Vc

        return dq

    # output dqc
    def stateSs(self, qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt,
                pg: np.ndarray, zg):

        Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)
        Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)

        ref_pt_mat = matlab.double(self.ref_pt)
        ref_pg_mat = matlab.double(self.ref_pg)
        Pcg_mat = matlab.double(Pcg.tolist())
        Pct_mat = matlab.double(Pct.tolist())
        qc_mat = matlab.double(qc.tolist())
        dqc_mat = matlab.double(dqc.tolist())
        pg_mat = matlab.double(pg.tolist())
        pt_mat = matlab.double(pt.tolist())
        qlim_mat = matlab.double(self.q_lim.T.tolist())
        dqlim_mat = matlab.double(self.dq_lim.T.tolist())
        resolution_mat = matlab.double(self.resolution.tolist())

        dq = self.eng.optimization_Ss(resolution_mat, ref_pt_mat, ref_pg_mat, qlim_mat, dqlim_mat,
                                      Pcg_mat, Pct_mat, qc_mat, dqc_mat, pg_mat, pt_mat)
        return dq

    def stateSa(self, qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt,
                pg: np.ndarray, zg):

        Jcc = jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)
        Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ Jcc

        Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc

        ref_pt_mat = matlab.double(self.ref_pt)
        ref_pg_mat = matlab.double(self.ref_pg)
        Pcg_matlab = matlab.double(Pcg.tolist())
        Pct_matlab = matlab.double(Pct.tolist())
        Jcc_matlab = matlab.double(Jcc.tolist())
        qc_matlab = matlab.double(qc.tolist())
        dqc_matlab = matlab.double(dqc.tolist())
        pg_matlab = matlab.double(pg.tolist())
        pt_matlab = matlab.double(pt.tolist())
        qlim_mat = matlab.double(self.q_lim.T.tolist())
        dqlim_mat = matlab.double(self.dq_lim.T.tolist())
        resolution_mat = matlab.double(self.resolution.tolist())

        # T = forwardKinematics(Mat_S_space_camera, Mat_M_camera, qc, 'space')
        # r = R.from_matrix(T[:3,:3])
        # r.as_rotvec()

        dq = self.eng.optimization_Sa(resolution_mat, ref_pt_mat, ref_pg_mat, qlim_mat, dqlim_mat, Pcg_matlab,
                                      Pct_matlab, Jcc_matlab, qc_matlab, dqc_matlab, pg_matlab, pt_matlab)
        return dq

    def stateSc(self, qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pg: np.ndarray, zg,
                pg_seg: list):

        Jcc = jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)
        Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc

        Pcg_vertices = [(getIM(p, z) @ adjointMatrix(T_ext) @ Jcc).tolist() for p, z in pg_seg]
        pgv = [p.tolist() for p, _ in pg_seg]

        ref_pt_mat = matlab.double(self.ref_pt)
        ref_pg_mat = matlab.double(self.ref_pg)
        Pcg_mat = matlab.double(Pcg.tolist())
        Jcc_mat = matlab.double(Jcc.tolist())
        qc_mat = matlab.double(qc.tolist())
        dqc_mat = matlab.double(dqc.tolist())
        pg_mat = matlab.double(pg.tolist())
        Pcg_vertices_mat = matlab.double(Pcg_vertices)
        pgv_mat = matlab.double(pgv)
        qlim_mat = matlab.double(self.q_lim.T.tolist())
        dqlim_mat = matlab.double(self.dq_lim.T.tolist())
        resolution_mat = matlab.double(self.resolution.tolist())

        dq = self.eng.optimization_Sc(resolution_mat, ref_pt_mat, ref_pg_mat, qlim_mat, dqlim_mat, Pcg_mat,
                                      Jcc_mat, qc_mat, dqc_mat, pg_mat, Pcg_vertices_mat, pgv_mat)
        return dq

    def stateSo(self, qc: np.ndarray, qt: np.ndarray, dqc: np.ndarray, dqt: np.ndarray, pt: np.ndarray, zt,
                pg: np.ndarray, zg, po: np.ndarray, zo, po_seg: list):
        Jcc = jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)

        Pcg = getIM(pg, zg) @ adjointMatrix(T_ext) @ Jcc
        Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ Jcc
        Pco = getIM(po, zo) @ adjointMatrix(T_ext) @ Jcc

        Pco_vertices = [(getIM(p, z) @ adjointMatrix(T_ext) @ Jcc).tolist() for p, z in po_seg]
        pov = [p.tolist() for p, _ in po_seg]

        ref_pt_mat = matlab.double(self.ref_pt)
        ref_pg_mat = matlab.double(self.ref_pg)
        ref_po_mat = matlab.double(self.ref_po)
        Pcg_mat = matlab.double(Pcg.tolist())
        Pct_mat = matlab.double(Pct.tolist())
        Pco_mat = matlab.double(Pco.tolist())
        Jcc_mat = matlab.double(Jcc.tolist())
        qc_mat = matlab.double(qc.tolist())
        dqc_mat = matlab.double(dqc.tolist())
        pt_mat = matlab.double(pt.tolist())
        pg_mat = matlab.double(pg.tolist())
        po_mat = matlab.double(po.tolist())
        Pco_vertices_mat = matlab.double(Pco_vertices)
        pov_mat = matlab.double(pov)
        qlim_mat = matlab.double(self.q_lim.T.tolist())
        dqlim_mat = matlab.double(self.dq_lim.T.tolist())
        resolution_mat = matlab.double(self.resolution.tolist())

        dq = self.eng.optimization_So(resolution_mat, ref_pt_mat, ref_pg_mat, ref_po_mat, qlim_mat, dqlim_mat, Pct_mat,
                                      Pcg_mat, Pco_mat, Jcc_mat, qc_mat, dqc_mat, pt_mat, pg_mat, po_mat,
                                      Pco_vertices_mat, pov_mat)
        return dq

    def constraint_o(self, qc, qt, dqc, dqt, pt, dpt, zt, ps: np.ndarray):

        Jcc = jacobian_body(self.camArm.Mat_S_space, self.camArm.Mat_M, qc)

        # ret = False
        for i, (sa, sb) in enumerate(ps):
            pa, dpa, za = sa
            pb, dpb, zb = sb
            alpha = (pb - pa) @ (pb - pa)
            beta = 2 * (pa - pt) @ (pb - pa)
            gamma = (pa - pt) @ (pa - pt)
            alpha_p = 0.025 * (pb - pa) @ (dpb - dpa)
            beta_p = 0.025 * ((pa - pt) @ (dpb - dpa) + (pb - pa) @ (dpa - dpt))
            gamma_p = 0.025 * (pa - pt) @ (dpa - dpt)

            a = alpha + alpha_p
            b = beta + beta_p
            c = gamma + gamma_p
            print('a', a)
            print('b', b)
            print('c', c)

            if a < 0:
                if c >= a + b + c:
                    s_bar = 0
                else:
                    s_bar = 1
            else:  # a > 0
                s = - b / (2 * a)
                if s > 1:
                    s_bar = 1
                elif s < 0:
                    s_bar = 0
                else:
                    s_bar = s

            print("s_bar:", s_bar)
            min = a * s_bar ** 2 + b * s_bar + c
            print('min:', min)
            if min < 0:
                return True

            # g = -(alpha * s_bar * s_bar + beta * s_bar + gamma)
            # dq = np.hstack((dqc, dqt))
            # Pca = getIM(pa, za) @ adjointMatrix(T_ext) @ Jcc
            # Pcb = getIM(pb, zb) @ adjointMatrix(T_ext) @ Jcc
            # Pct = getIM(pt, zt) @ adjointMatrix(T_ext) @ Jcc
            # n = np.zeros((6, 6), dtype=np.float64)
            # T = forwardKinematics(Mat_S_space_camera, Mat_M_camera, qc)
            # n[:3, :3] = -T[:3, :3]  # Rc
            # Ptt = getIM(pt, zt) @ adjointMatrix(T_ext) @ n @ jacobian(Mat_S_space_tool, qt)
            #
            # Ec = 0.025 * ((pb - pa) @ (Pcb - Pca) * s_bar * s_bar + (
            #             (pb - pa) @ (Pca - Pct) + (pa - pt) @ (Pca - Pct)) * s_bar + (pa - pt) @ (Pca - Pct))
            # Et = -0.025 * ((pb - pa) * s_bar + (pa - pt)) @ Ptt
            #
            # E = np.hstack((Ec, Et))
            #
            # result = E @ dq - g
            # if result < 0:
            #     print('constraint active:', i)
            #     return True

        return False
