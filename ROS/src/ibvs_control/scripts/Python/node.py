#!/usr/bin/python3
import rospy
import numpy as np
import time

import optimization
from ibvs_control.srv import OptimIBVS, OptimIBVSResponse, OptimIBVSRequest
from std_msgs.msg import Float64


class BackendNode:
    def __init__(self) -> None:
        rospy.init_node('backend_ros_node')
        self.serv_states = rospy.Service('optim_ibvs', OptimIBVS, self.service_callback)
        self.state = 'Ss'

    def service_callback(self, req: OptimIBVSRequest):
        qt = np.asarray([i.data for i in req.qt], dtype=np.float64)
        dqt = np.asarray([i.data for i in req.dqt], dtype=np.float64)
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
            result = optimization.stateSs(qc, qt, dqc, dqt, pt, zt, pg, zg)[0]
            e1 = np.linalg.norm(optimization.Ref_pt - pt)
            e2 = np.linalg.norm(optimization.Ref_pg - pg)
            print('e1:', e1)
            print('e2:', e2)
            if e1 < 10 and e2 < 10:
                self.state = 'Sa'
                # phi_start = optimization.getPhiStart()
        elif self.state == 'Sa':
            result = optimization.stateSa(qc, qt, dqc, dqt, pt, zt, pg, zg)[0]
            e1 = np.linalg.norm(optimization.Ref_pt - pt)
            e2 = np.linalg.norm(optimization.Ref_pg - pg)
            print('e1:', e1)
            print('e2:', e2)
            print(area_g)
            if e1 < 8 and area_g >= 60000:
                self.state = 'Sc'
            # if optimization.constraint_o():
            #      self.state = 'So'
        elif self.state == 'Sc':
            result = optimization.stateSc(qc, qt, dqc, dqt, pg, zg, pg_seg)[0]
            e1 = np.linalg.norm(optimization.Ref_pg - pg)
            print('e1:', e1)
            if pg[0] > optimization.RESOLUTION[0] or pg[1] > optimization.RESOLUTION[1]:
                self.state = 'Ss'
            # if optimization.constraint_o():
            #      self.state = 'So'
        elif self.state == 'So':
            result = optimization.stateSo(qc, qt, dqc, dqt, pg, zg, pg_seg)[0]
            e1 = np.linalg.norm(optimization.Ref_po - po)
            # if not optimization.constraint_o() and e1 < 20:
            #     self.state = 'Sa'

        print('result\n', result)
        print('opt time:', time.time() - t1)
        print()

        response = OptimIBVSResponse()
        response.dqc_next = [Float64(i) for i in result]  # list(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        return response

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = BackendNode()
    node.run()
