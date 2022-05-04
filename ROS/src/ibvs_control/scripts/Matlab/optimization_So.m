function res = optimization_So(Pct, Pcg, Pco, Jcc, qc, dqc, pt, pg, po, Pco_vertices, pov)
    Ref_pg = [654.0, 1881.0]';
    Ref_pt = [1691.0, 302.0]';
    Ref_po = [1487.0, 1387.0]';%1621.2097168, 989.36187744
    qc = qc';
    dqc = dqc';
    pg = pg';
    po = po';
    pt = pt';

    q_limit = [-pi, -2.41, -2.66, -pi, -2.23, -pi;
                pi, 2.41, 2.66, pi, 2.23, pi]';
    %%
    dq0 = optimvar('dq0');
    dq1 = optimvar('dq1');
    dq2 = optimvar('dq2');
    dq3 = optimvar('dq3');
    dq4 = optimvar('dq4');
    dq5 = optimvar('dq5');

    dq = [dq0,dq1,dq2,dq3,dq4,dq5]';
    dpg = Pcg*dq;
    dpt = Pct*dq;
    dpo = Pco*dq;
    po_next = po + dpo * 0.05;
    pg_next = pg + dpg * 0.05;
    pt_next = pt + dpt * 0.05;

    Vcc = Jcc * dq;
% 
%     Pco1 = squeeze(Pco_vertices(1,:,:));
%     Pco2 = squeeze(Pco_vertices(2,:,:));
%     Pco3 = squeeze(Pco_vertices(3,:,:));
%     Pco4 = squeeze(Pco_vertices(4,:,:));
%     dpo1 = Pco1 * dq;
%     dpo2 = Pco2 * dq;
%     dpo3 = Pco3 * dq;
%     dpo4 = Pco4 * dq;
%     po1_next = pov(1,:)' + dpo1 * 0.05;
%     po2_next = pov(2,:)' + dpo2 * 0.05;
%     po3_next = pov(3,:)' + dpo3 * 0.05;
%     po4_next = pov(4,:)' + dpo4 * 0.05;


%     det1 = (po2_next(1) - po1_next(1)) * (po2_next(2) - po3_next(2)) - (po2_next(2) - po1_next(2)) * (po2_next(1) - po3_next(1));
%     det2 = (po4_next(1) - po1_next(1)) * (po4_next(2) - po3_next(2)) - (po4_next(2) - po1_next(2)) * (po4_next(1) - po3_next(1));
%     s13 = norm(po1_next - po3_next);
%     d1 = norm(det1) / norm(po1_next - po3_next);
%     d2 = norm(det2) / norm(po1_next - po3_next);
%     area_o = 0.5*(d1 + d2)*s13;

    e1 = dpt.^2;
    e2 = dpg.^2;
    e3 = (Ref_po - po_next).^2;
%     e4 = area_o;%^2;
    e5 = Vcc(6)^2;
    e6 = (Ref_pg - pg_next).^2;
%     e7 = (Ref_pt - pt_next).^2;
%     e8 = Vcc(3)^2;

    prob = optimproblem;
    prob.Objective =  e6(1) + e6(2) + e3(1) + e3(2) + e5;%+ 1000*e8 e7(1) + e7(2) +  + e50.01*e1(1) + 0.01*e1(2) + 0.01*e2(1) + 0.01*e2(2) + e5 0.001*+ e4 0.0001*10000*
    prob.Constraints.cons1 = dq <= [1.3963 1.3963 1.3963 1.2218 1.2218 1.2218]';
    prob.Constraints.cons2 = dq >= [-1.3963 -1.3963 -1.3963 -1.2218 -1.2218 -1.2218]';

    prob.Constraints.cons3 = qc + dq * 0.05 <= q_limit(:, 2);
    prob.Constraints.cons4 = qc + dq * 0.05 >= q_limit(:, 1);
%
    prob.Constraints.cons5 = pg_next <= [2064.0 2096.0]';
    prob.Constraints.cons6 = pg_next >= [0.0 0.0]';

    x0.dq0 = dqc(1);
    x0.dq1 = dqc(2);
    x0.dq2 = dqc(3);
    x0.dq3 = dqc(4);
    x0.dq4 = dqc(5);
    x0.dq5 = dqc(6);

    [sol,fval,exitflag,output,lambda] = solve(prob, x0, 'Solver', 'fmincon');%, 

    res = [sol.dq0, sol.dq1, sol.dq2, sol.dq3, sol.dq4, sol.dq5]*0.1;
    Vcc_ = Jcc*res';
%     e5_ = Vcc_(6)^2

%     dpo1_ = Pco1 * res';
%     dpo2_ = Pco2 * res';
%     dpo3_ = Pco3 * res';
%     dpo4_ = Pco4 * res';
%     po1_next_ = pov(1,:)' + dpo1_ * 0.05;
%     po2_next_ = pov(2,:)' + dpo2_ * 0.05;
%     po3_next_ = pov(3,:)' + dpo3_ * 0.05;
%     po4_next_ = pov(4,:)' + dpo4_ * 0.05;
% 
%     det1_ = (po2_next_(1) - po1_next_(1)) * (po2_next_(2) - po3_next_(2)) - (po2_next_(2) - po1_next_(2)) * (po2_next_(1) - po3_next_(1));
%     det2_ = (po4_next_(1) - po1_next_(1)) * (po4_next_(2) - po3_next_(2)) - (po4_next_(2) - po1_next_(2)) * (po4_next_(1) - po3_next_(1));
%     s13_ = norm(po1_next_ - po3_next_);
%     d1_ = norm(det1_) / norm(po1_next_ - po3_next_);
%     d2_ = norm(det2_) / norm(po1_next_ - po3_next_);
%     area_o_ = 0.5*(d1_ + d2_)*s13_;
%     e4_ = 0.001*area_o_

    dpg_ = Pcg*res';
    dpt_ = Pct*res';
    e1_ = dpg_.^2;
    e2_ = dpt_.^2;
%     e1_
%     e2_

    dpo_ = Pco*res';
    po_next_ = po + dpo_ * 0.05;
    e3_ = (Ref_po - po_next_).^2   

    dpg_ = Pcg*res';
    pg_next_ = pg + dpg_ * 0.05;
    e6_ = (Ref_pg - pg_next_).^2  

    dpt_ = Pct*res';
    pt_next_ = pt + dpt_ * 0.05;
    e7_ = (Ref_pt - pt_next_).^2  
end