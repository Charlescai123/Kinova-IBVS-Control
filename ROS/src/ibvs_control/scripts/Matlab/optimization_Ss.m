function res = optimization_Ss(resolution, Ref_pt, Ref_pg, q_limit, dq_limit, Pcg, Pct, qc, dqc, pg, pt)
    %Ref_pt = [520, 110]';
    %Ref_pg = [180, 396]';
    % Ref_pt = [1691.0, 302.0]';
    % Ref_pg = [854.0, 1681.0]';
    Ref_pt = Ref_pt';
    Ref_pg = Ref_pg';

    qc = qc';
    dqc = dqc';
    pg = pg';
    pt = pt';
    
    dq0 = optimvar('dq0');
    dq1 = optimvar('dq1');
    dq2 = optimvar('dq2');
    dq3 = optimvar('dq3');
    dq4 = optimvar('dq4');
    dq5 = optimvar('dq5');
   
    dq = [dq0,dq1,dq2,dq3,dq4,dq5]';
    dpg = Pcg*dq;
    dpt = Pct*dq;
    pg_next = pg + dpg * 0.05;
    pt_next = pt + dpt * 0.05;
    
    e1 = (Ref_pg - pg_next).^2;
    e2 = (Ref_pt - pt_next).^2;
    
    prob = optimproblem;
    prob.Objective = e1(1)+e1(2) + e2(1)+e2(2);

    prob.Constraints.cons1 = dq >= dq_limit(:, 1);
    prob.Constraints.cons2 = dq <= dq_limit(:, 2);

    prob.Constraints.cons3 = qc + dq * 0.05 >= q_limit(:, 1);
    prob.Constraints.cons4 = qc + dq * 0.05 <= q_limit(:, 2);

    prob.Constraints.cons5 = pg_next <= resolution';
    prob.Constraints.cons6 = pg_next >= [0.0 0.0]';

    x0.dq0 = dqc(1);
    x0.dq1 = dqc(2);
    x0.dq2 = dqc(3);
    x0.dq3 = dqc(4);
    x0.dq4 = dqc(5);
    x0.dq5 = dqc(6);

    [sol,fval,exitflag,output,lambda] = solve(prob, x0);%, 'Solver', 'fmincon'
%     Vc_ = mat3*[sol.dq0, sol.dq1, sol.dq2, sol.dq3, sol.dq4, sol.dq5]';
%     e_ = Vc_ref - Vc_
    res = [sol.dq0, sol.dq1, sol.dq2, sol.dq3, sol.dq4, sol.dq5]*0.1;
    
%     lambda.Constraints.cons1
%     lambda.Constraints.cons2
%     lambda.Constraints.cons3
%     lambda.Constraints.cons4
%     lambda.Constraints.cons5
%     lambda.Constraints.cons6
%     dpg_ = Pcg * res';
%     dpt_ = Pct * res';
%     e_ = Ref_pg - (pg + dpg_ * 0.05)
%     e_ = Ref_pt - (pt + dpt_ * 0.05)
end
