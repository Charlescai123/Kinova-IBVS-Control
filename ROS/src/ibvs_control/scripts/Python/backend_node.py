#!/usr/bin/python3
import threading

import rospy
import numpy as np
import matplotlib.pyplot as plt
import time

from matplotlib.animation import FuncAnimation

from optimization import *
from ibvs_control.srv import OptimIBVS, OptimIBVSResponse, OptimIBVSRequest
from std_msgs.msg import Float64


class BackendNode:
    def __init__(self) -> None:
        rospy.init_node('backend_ros_node')
        self.serv_states = rospy.Service('optim_ibvs', OptimIBVS, self.service_callback)
        self.optimizer = StateMachineOptimizer()
        self.state = 'Ss'
        self.total_time = [time.process_time()]
        self.plot_goal1 = [0]
        self.plot_goal2 = [0]
        self.plot_goal3 = [0]
        self.plot_goal4 = [0]

        # t = threading.Thread(target=self.plot_evaluation, name='LoopThread')
        # t.start()

    def plot_evaluation(self):
        # x = np.linspace(0, 2 * np.pi, 20)

        # plt.plot(self.total_time, self.plot_goal, 'ro-', label='sinx')

        plt.ion()
        # First set up the figure, the axis, and the plot element we want to
        fig = plt.figure(figsize=(6, 4))
        ax = plt.axes()
        line, = ax.plot([], [], lw=2)

        line.set_data(self.total_time, self.plot_goal)
        # plt.plot(x, y, 'b*--', label='cosx')

        plt.title('plot curve', fontsize=25)  # 标题
        # plt.xlim(-1, 7)  # x轴范围
        # plt.ylim(-1.5, 1.5)  # y轴范围
        plt.xlabel('x', fontsize=20)  # x轴标签
        plt.ylabel('y', fontsize=20)  # y轴标签
        plt.legend(loc='best')  # 图例

        plt.pause(0.1)
        plt.show()
        plt.clf()

        # continue

    def service_callback(self, req: OptimIBVSRequest):
        qm = np.asarray([i.data for i in req.qm], dtype=np.float64)
        dqm = np.asarray([i.data for i in req.dqm], dtype=np.float64)
        qc = np.asarray([i.data for i in req.qc], dtype=np.float64)
        dqc = np.asarray([i.data for i in req.dqc], dtype=np.float64)
        # print('qt\n', qt)
        # print('dqt\n', dqt)
        # print('qc\n', qc)
        # print('dqc\n', dqc)

        pt = np.asarray([i.data for i in req.pt.position], dtype=np.float64)
        dpt = np.asarray([i.data for i in req.pt.velocity], dtype=np.float64)
        zt = req.pt.depth.data

        # print('pt\n', pt)
        # print('dpt\n', dpt)
        # print('zt\n', zt)

        pg = np.asarray([i.data for i in req.pg.position], dtype=np.float64)
        dpg = np.asarray([i.data for i in req.pg.velocity], dtype=np.float64)
        zg = req.pg.depth.data
        # print('pg\n', pg)
        # print('dpg\n', dpg)
        # print('zg\n', zg)

        po = np.asarray([i.data for i in req.po.position], dtype=np.float64)
        dpo = np.asarray([i.data for i in req.po.velocity], dtype=np.float64)
        zo = req.pg.depth.data
        # print('po\n', po)
        # print('dpo\n', dpo)
        # print('zo\n', zo)

        area_g = req.area_g.data
        area_o = req.area_o.data
        # print(area_o)
        # print('area_g\n', area_g)
        # print('area_o\n', area_o)

        ps = []
        l = len(req.po_seg)
        for i in range(l + 1):
            pa, pb = req.po_seg[i % l], req.po_seg[(i + 1) % l]
            ps.append(((np.asarray([i.data for i in pa.position], dtype=np.float64),
                        np.asarray([i.data for i in pa.velocity], dtype=np.float64),
                        pa.depth.data),
                       (np.asarray([i.data for i in pb.position], dtype=np.float64),
                        np.asarray([i.data for i in pb.velocity], dtype=np.float64),
                        pb.depth.data)))

        pg_seg = []
        l = len(req.pg_seg)
        for i in range(l):
            p = req.pg_seg[i]
            pg_seg.append((np.asarray([i.data for i in p.position]), p.depth.data))

        po_seg = []
        l = len(req.po_seg)
        for i in range(l):
            p = req.po_seg[i]
            po_seg.append((np.asarray([i.data for i in p.position]), p.depth.data))

        print('State:', self.state)
        t1 = time.time()
        if self.state == 'Ss':
            result = self.optimizer.stateSs(qc, qm, dqc, dqm, pt, zt, pg, zg)[0]
            e1 = np.linalg.norm(self.optimizer.ref_pt - pt)
            e2 = np.linalg.norm(self.optimizer.ref_pg - pg)

            print('e1:', e1)
            print('e2:', e2)
            if e1 < 10 and e2 < 10:
                self.state = 'Sa'
                # phi_start = optimization.getPhiStart()
        elif self.state == 'Sa':
            result = self.optimizer.stateSa(qc, qm, dqc, dqm, pt, zt, pg, zg)[0]
            e1 = np.linalg.norm(self.optimizer.ref_pt - pt)
            e2 = np.linalg.norm(self.optimizer.ref_pg - pg)

            print('e1:', e1)
            print('e2:', e2)
            print(area_g)
            if e1 < 20 and area_g >= 30000:
                self.state = 'Sc'
            if area_o == 1.0:
                self.state = 'So'
        elif self.state == 'Sc':
            result = self.optimizer.stateSc(qc, qm, dqc, dqm, pg, zg, pg_seg)[0]
            e1 = np.linalg.norm(self.optimizer.ref_pg - pg)
            print('e1:', e1)
            if pg[0] > RESOLUTION[0] or pg[1] > RESOLUTION[1]:
                self.state = 'Ss'
            # if area_o == 0.0:
            #      self.state = 'So'
        elif self.state == 'So':
            result = self.optimizer.stateSo(qc, qm, dqc, dqm, pt, zt, pg, zg, po, zo, po_seg)[0]
            e1 = np.linalg.norm(self.optimizer.ref_po - po)
            e2 = np.linalg.norm(self.optimizer.ref_pg - pg)
            # print(po)
            print('e1:', e1)
            print('e2:', e2)
            if area_o == 0.0:
                self.state = 'Sa'

        # print('result\n', result)
        # if optimization.constraint_o(qc, qt, dqc, dqt, pt, dpt, zt, ps):
        #     print('--->')
        # print('--->', optimization.constraint_o(qc, qt, dqc, dqt, pt, dpt, zt, ps))
        print('opt time:', time.time() - t1)
        print()

        response = OptimIBVSResponse()
        response.dqc_next = [Float64(i) for i in result]  # list(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])) result

        # self.plot_evaluation()

        return response

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = BackendNode()
    node.run()
