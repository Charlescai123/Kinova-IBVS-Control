function res = optimization_Sc(Pcg, Jcc, qc, dqc, pg, Pcg_vertices, pgv)
    Ref_pg = [854.0, 1681.0]';
    Ref_area_g = 30000;
    qc = qc';
    dqc = dqc';
    pg = pg';

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
    pg_next = pg + dpg * 0.05;

    Vcc = Jcc * dq;

    Pcg1 = squeeze(Pcg_vertices(1,:,:));
    Pcg2 = squeeze(Pcg_vertices(2,:,:));
    Pcg3 = squeeze(Pcg_vertices(3,:,:));
    Pcg4 = squeeze(Pcg_vertices(4,:,:));
    dpg1 = Pcg1 * dq;
    dpg2 = Pcg2 * dq;
    dpg3 = Pcg3 * dq;
    dpg4 = Pcg4 * dq;
    pg1_next = pgv(1,:)' + dpg1 * 0.05;
    pg2_next = pgv(2,:)' + dpg2 * 0.05;
    pg3_next = pgv(3,:)' + dpg3 * 0.05;
    pg4_next = pgv(4,:)' + dpg4 * 0.05;

    
    det1 = (pg2_next(1) - pg1_next(1)) * (pg2_next(2) - pg3_next(2)) - (pg2_next(2) - pg1_next(2)) * (pg2_next(1) - pg3_next(1));
    det2 = (pg4_next(1) - pg1_next(1)) * (pg4_next(2) - pg3_next(2)) - (pg4_next(2) - pg1_next(2)) * (pg4_next(1) - pg3_next(1));
    s13 = norm(pg1_next - pg3_next);
    d1 = norm(det1) / norm(pg1_next - pg3_next);
    d2 = norm(det2) / norm(pg1_next - pg3_next);
    area_g = 0.5*(d1 + d2)*s13;
%     s_ = norm(pgv(1,:)' - pgv(3,:)');
%     d1_ = norm(det([pgv(2,:)' - pgv(1,:)', pgv(2,:)' - pgv(3,:)'])) / norm(pgv(1,:)' - pgv(3,:)');
%     d2_ = norm(det([pgv(4,:)' - pgv(1,:)', pgv(4,:)' - pgv(3,:)'])) / norm(pgv(1,:)' - pgv(3,:)');
%     area_g_ = round(0.5*(d1_ + d2_)*s_)

    e1 = (Ref_pg - pg_next).^2;
    e2 = (Ref_area_g - area_g)^2;
    e3 = Vcc(3)^2;

    prob = optimproblem;
    prob.Objective = e1(1) + e1(2) + 0.0001*e2 + 10000*e3;%
    prob.Constraints.cons1 = dq <= [1.3963 1.3963 1.3963 1.2218 1.2218 1.2218]'*0.5;
    prob.Constraints.cons2 = dq >= [-1.3963 -1.3963 -1.3963 -1.2218 -1.2218 -1.2218]'*0.5;

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

    [sol,fval,exitflag,output,lambda] = solve(prob, x0);%, 'Solver', 'fmincon'

    res = [sol.dq0, sol.dq1, sol.dq2, sol.dq3, sol.dq4, sol.dq5]*0.05;
%     Vcc_ = Jcc*res';
%     e3_ = 10000 * Vcc_(3)^2
% 
%     dpg1_ = Pcg1 * res';
%     dpg2_ = Pcg2 * res';
%     dpg3_ = Pcg3 * res';
%     dpg4_ = Pcg4 * res';
%     pg1_next_ = pgv(1,:)' + dpg1_ * 0.05;
%     pg2_next_ = pgv(2,:)' + dpg2_ * 0.05;
%     pg3_next_ = pgv(3,:)' + dpg3_ * 0.05;
%     pg4_next_ = pgv(4,:)' + dpg4_ * 0.05;
    
%     det1_ = (pg2_next_(1) - pg1_next_(1)) * (pg2_next_(2) - pg3_next_(2)) - (pg2_next_(2) - pg1_next_(2)) * (pg2_next_(1) - pg3_next_(1));
%     det2_ = (pg4_next_(1) - pg1_next_(1)) * (pg4_next_(2) - pg3_next_(2)) - (pg4_next_(2) - pg1_next_(2)) * (pg4_next_(1) - pg3_next_(1));
%     s13_ = norm(pg1_next_ - pg3_next_);
%     d1_ = norm(det1_) / norm(pg1_next_ - pg3_next_);
%     d2_ = norm(det2_) / norm(pg1_next_ - pg3_next_);
%     area_g_ = 0.5*(d1_ + d2_)*s13_
%     e2_ = 0.000001*(Ref_area_g - area_g_)^2

%     dpg_ = Pcg*res';
%     pg_next_ = pg + dpg_ * 0.05;
%     e1_ = (Ref_pg - pg_next_).^2
end
