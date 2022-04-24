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
        l = len(req.ps)
        for i in range(l + 1):
            pa, pb = req.ps[i % l], req.ps[(i + 1) % l]
            ps.append(((np.asarray([i.data for i in pa.position], dtype=np.float64),
                        np.asarray([i.data for i in pa.velocity], dtype=np.float64),
                        pa.depth.data),
                       (np.asarray([i.data for i in pb.position], dtype=np.float64),
                        np.asarray([i.data for i in pb.velocity], dtype=np.float64),
                        pb.depth.data)))
        # print(ps)
        t1 = time.time()
        result = optimization.stateSs(qc, qt, dqc, dqt, pt, zt, pg, zg)[0]
        # result = optimization.feedbackLaw(qc, dqc, pg, zg)
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
