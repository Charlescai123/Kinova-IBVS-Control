close; clear; clc
syms th1 th2 th3 th4 th5 th6 real
syms dth1 dth2 dth3 dth4 dth5 dth6 real
syms ddth1 ddth2 ddth3 ddth4 ddth5 ddth6 real
% syms m1 m2 m3 m4 m5 m6
% syms l1 l2 l3 l4 l5 l6 I1 I2 I3 I4 I5 I6 r1 r2 r3 r4 r5 r6;
syms u1 u2 u3 u4 u5 u6 real
g = 9.8;

q = [th1; th2; th3; th4; th5; th6];
dq = [dth1; dth2; dth3; dth4; dth5; dth6];
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5; ddth6];

u = [u1; u2; u3; u4; u5; u6];

m1 = 1.377;
m2 = 1.262;
m3 = 0.930;
m4 = 0.678;
m5 = 0.678;
m6 = 0.5;
m = [m1 m2 m3 m4 m5 m6]';

I1 = [0.004570, 0.000001, 0.000002; 
      0.000001, 0.004831, 0.000448; 
      0.000002, 0.000448, 0.001409];
  
I2 = [0.046752, -0.000009, 0.000000; 
      -0.000009, 0.000850, -0.000098; 
      0.000000, -0.000098, 0.047188];
  
I3 = [0.008292, -0.000001, 0.000000; 
      -0.000001, 0.000628, 0.000432; 
      0.000000, 0.000432, 0.008464];
  
I4 = [0.001645, 0.000000, 0.000000; 
      0.000000, 0.001666, -0.000234; 
      0.000000, -0.000234, 0.000389];
 
I5 = [0.001685, 0.000000, 0.000000; 
      0.000000, 0.000400, 0.000255; 
      0.000000, 0.000255, 0.001696];
  
I6 = [0.000587, 0.000003, 0.000003; 
      0.000003, 0.000369, -0.000118; 
      0.000003, -0.000118, 0.000609];

% I1 = [0.004570, 0, 0; 
%       0, 0.004831, 0; 
%       0, 0, 0.001409];
%   
% I2 = [0.046752, 0, 0; 
%       0, 0.000850, 0; 
%       0, 0, 0.047188];
%   
% I3 = [0.008292, 0, 0; 
%       0, 0.000628, 0; 
%       0, 0, 0.008464];
%   
% I4 = [0.001645, 0, 0; 
%       0, 0.001666, 0; 
%       0, 0, 0.000389];
%  
% I5 = [0.001685, 0, 0; 
%       0, 0.000400, 0; 
%       0, 0, 0.001696];
%   
% I6 = [0.000587, 0, 0; 
%       0, 0.000369, 0; 
%       0, 0, 0.000609];

% A1 = DH(0, 0, pi, 0)*DH(0, -(0.15643+0.12838), pi/2, th1);
% A2 = DH(0.410, -0.00538, pi, th2 - pi/2);
% A3 = DH(0, -0.00638, pi/2, th3 - pi/2);
% A4 = DH(0, -(0.20843+0.10593), pi/2, th4 + pi);
% A5 = DH(0, 0, pi/2, th5 + pi);
% A6 = DH(0, -(105.93 + 61.53), pi, th6 + pi);

Tb1 = [cos(th1), -sin(th1), 0, 0;
       -sin(th1), -cos(th1), 0, 0;
       0, 0, -1, 0.1564;
       0, 0, 0, 1];
   
T12 = [cos(th2), -sin(th2), 0, 0;
       0, 0, -1, 0.0054;
       sin(th2), cos(th2), 0, -0.1284;
       0, 0, 0, 1];
   
T23 = [cos(th3), -sin(th3), 0, 0;
       -sin(th3), -cos(th3), 0, -0.410;
       0, 0, -1, 0;
       0, 0, 0, 1];

T34 = [cos(th4), -sin(th4), 0, 0;
       0, 0, -1, 0.2084;
       sin(th4), cos(th4), 0, -0.0064;
       0, 0, 0, 1];
   
T45 = [cos(th5), -sin(th5), 0, 0;
       0, 0, 1, 0;
       -sin(th5), -cos(th5), 0, -0.1059;
       0, 0, 0, 1];

T56 = [cos(th6), -sin(th6), 0, 0;
       0, 0, -1, 0.1059;
       sin(th6), cos(th6), 0, 0;
       0, 0, 0, 1];
   
T6end = [-1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, -1, -0.0615;
         0, 0, 0, 1];


T00 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T01 = Tb1;
T02 = Tb1*T12;
T03 = Tb1*T12*T23;
T04 = Tb1*T12*T23*T34;
T05 = Tb1*T12*T23*T34*T45;
T06 = Tb1*T12*T23*T34*T45*T56;
% T01 = A1;
% T02 = A1*A2;
% T03 = A1*A2*A3;
% T04 = A1*A2*A3*A4;
% T05 = A1*A2*A3*A4*A5;
% T06 = A1*A2*A3*A4*A5*A6;

Tcom1 = [1 0 0 -0.000023; 0 1 0 -0.010364; 0 0 1 -0.073360; 0 0 0 1];
Tcom2 = [1 0 0 0.000035; 0 1 0 -0.208207; 0 0 1 -0.018890; 0 0 0 1];
Tcom3 = [1 0 0 0.000018; 0 1 0 0.076168; 0 0 1 -0.013970; 0 0 0 1];
Tcom4 = [1 0 0 -0.000001; 0 1 0 0.008466; 0 0 1 -0.062937; 0 0 0 1];
Tcom5 = [1 0 0 -0.000001; 0 1 0 0.046429; 0 0 1 -0.008704; 0 0 0 1];
Tcom6 = [1 0 0 0.000281; 0 1 0 0.011402; 0 0 1 -0.029798; 0 0 0 1];

T0com1 = T01*Tcom1;
T0com2 = T02*Tcom2;
T0com3 = T03*Tcom3;
T0com4 = T04*Tcom4;
T0com5 = T05*Tcom5;
T0com6 = T06*Tcom6;

% double(subs(T0com1, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))
% double(subs(T0com2, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))
% double(subs(T0com3, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))
% double(subs(T0com4, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))
% double(subs(T0com5, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))
% double(subs(T0com6, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]))

O00 = T00([1,2,3], 4);
O10 = T01([1,2,3], 4);
O20 = T02([1,2,3], 4);
O30 = T03([1,2,3], 4);
O40 = T04([1,2,3], 4);
O50 = T05([1,2,3], 4);
O60 = T06([1,2,3], 4);

Ocom10 = T0com1([1,2,3], 4);
Ocom20 = T0com2([1,2,3], 4);
Ocom30 = T0com3([1,2,3], 4);
Ocom40 = T0com4([1,2,3], 4);
Ocom50 = T0com5([1,2,3], 4);
Ocom60 = T0com6([1,2,3], 4);

Z00 = T00([1,2,3], 3);
Z10 = T01([1,2,3], 3);
Z20 = T02([1,2,3], 3);
Z30 = T03([1,2,3], 3);
Z40 = T04([1,2,3], 3);
Z50 = T05([1,2,3], 3);
% Z60 = T06([1,2,3], 3);

N = [0;0;0];
% J mat of link6(end effector), com
J6v1 = cross(Z00, (Ocom60 - O00));
J6v2 = cross(Z10, (Ocom60 - O10));
J6v3 = cross(Z20, (Ocom60 - O20));
J6v4 = cross(Z30, (Ocom60 - O30));
J6v5 = cross(Z40, (Ocom60 - O40));
J6v6 = cross(Z50, (Ocom60 - O50));

J6w1 = Z00;
J6w2 = Z10;
J6w3 = Z20;
J6w4 = Z30;
J6w5 = Z40;
J6w6 = Z50;

J6 = [J6v1, J6v2, J6v3, J6v4, J6v5, J6v6;
      J6w1, J6w2, J6w3, J6w4, J6w5, J6w6];
J6v = [J6v1, J6v2, J6v3, J6v4, J6v5, J6v6];
J6w = [J6w1, J6w2, J6w3, J6w4, J6w5, J6w6];

% J mat of link5, com
J5v1 = cross(Z00, (Ocom50 - O00));
J5v2 = cross(Z10, (Ocom50 - O10));
J5v3 = cross(Z20, (Ocom50 - O20));
J5v4 = cross(Z30, (Ocom50 - O30));
J5v5 = cross(Z40, (Ocom50 - O40));

J5w1 = Z00;
J5w2 = Z10;
J5w3 = Z20;
J5w4 = Z30;
J5w5 = Z40;

J5 = [J5v1, J5v2, J5v3, J5v4, J5v5, N;
      J5w1, J5w2, J5w3, J5w4, J5w5, N];

J5v = [J5v1, J5v2, J5v3, J5v4, J5v5, N];
J5w = [J5w1, J5w2, J5w3, J5w4, J5w5, N];

% J mat of link4, com
J4v1 = cross(Z00, (Ocom40 - O00));
J4v2 = cross(Z10, (Ocom40 - O10));
J4v3 = cross(Z20, (Ocom40 - O20));
J4v4 = cross(Z30, (Ocom40 - O30));

J4w1 = Z00;
J4w2 = Z10;
J4w3 = Z20;
J4w4 = Z30;

J4 = [J4v1, J4v2, J4v3, J4v4, N, N;
      J4w1, J4w2, J4w3, J4w4, N, N];
J4v = [J4v1, J4v2, J4v3, J4v4, N, N];
J4w = [J4w1, J4w2, J4w3, J4w4, N, N];

% J mat of link3, com
J3v1 = cross(Z00, (Ocom30 - O00));
J3v2 = cross(Z10, (Ocom30 - O10));
J3v3 = cross(Z20, (Ocom30 - O20));

J3w1 = Z00;
J3w2 = Z10;
J3w3 = Z20;

J3 = [J3v1, J3v2, J3v3, N, N, N;
      J3w1, J3w2, J3w3, N, N, N];
J3v = [J3v1, J3v2, J3v3, N, N, N];
J3w = [J3w1, J3w2, J3w3, N, N, N];

% J mat of link2, com
J2v1 = cross(Z00, (Ocom20 - O00));
J2v2 = cross(Z10, (Ocom20 - O10));

J2w1 = Z00;
J2w2 = Z10;

J2 = [J2v1, J2v2, N, N, N, N;
      J2w1, J2w2, N, N, N, N];
J2v = [J2v1, J2v2, N, N, N, N];
J2w = [J2w1, J2w2, N, N, N, N];

% J mat of link1, com
J1v1 = cross(Z00, (Ocom10 - O00));

J1w1 = Z00;

J1 = [J1v1, N, N, N, N, N;
      J1w1, N, N, N, N, N];
J1v = [J1v1, N, N, N, N, N];
J1w = [J1w1, N, N, N, N, N];

R0com1 = T0com1([1 2 3], [1 2 3]);
R0com2 = T0com2([1 2 3], [1 2 3]);
R0com3 = T0com3([1 2 3], [1 2 3]);
R0com4 = T0com4([1 2 3], [1 2 3]);
R0com5 = T0com5([1 2 3], [1 2 3]);
R0com6 = T0com6([1 2 3], [1 2 3]);

D1 = m1 * (J1v' * J1v) + J1w' * (R0com1 * I1 * R0com1') * J1w;
D2 = m2 * (J2v' * J2v) + J2w' * (R0com2 * I2 * R0com2') * J2w;
D3 = m3 * (J3v' * J3v) + J3w' * (R0com3 * I3 * R0com3') * J3w;
D4 = m4 * (J4v' * J4v) + J4w' * (R0com4 * I4 * R0com4') * J4w;
D5 = m5 * (J5v' * J5v) + J5w' * (R0com5 * I5 * R0com5') * J5w;
D6 = m6 * (J6v' * J6v) + J6w' * (R0com6 * I6 * R0com6') * J6w;

D = D1 + D2 + D3 + D4 + D5 + D6;
K = 0.5 * dq' * (D1 + D2 + D3 + D4 + D5 + D6) * dq;

% Ocom10 = T0com1([1,2,3], 4);
% Ocom20 = T0com2([1,2,3], 4);
% Ocom30 = T0com3([1,2,3], 4);
% Ocom40 = T0com4([1,2,3], 4);
% Ocom50 = T0com5([1,2,3], 4);
% Ocom60 = T0com6([1,2,3], 4);

P = m1*g*Ocom10(3,1) + m2*g*Ocom20(3,1) + m3*g*Ocom30(3,1) + m4*g*Ocom40(3,1) + m5*g*Ocom50(3,1) + m6*g*Ocom60(3,1);

% display(K);
% display(P);

L = K - P;

DL_Dq = jacobian(L, q);
% display(DL_Dq);
DL_Ddq = jacobian(L, dq);
% display(DL_Ddq);
DDL_DtDdq = jacobian(DL_Ddq, [q; dq]) * [dq; ddq];
% display(DDL_DtDdq);
EoM = DDL_DtDdq - DL_Dq' - u;

% EoM1 = DDL_DtDdq(1) - DL_Dq(1) - u1;
% EoM2 = DDL_DtDdq(2) - DL_Dq(2) - u2;
% EoM3 = DDL_DtDdq(3) - DL_Dq(3) - u3;
% EoM4 = DDL_DtDdq(4) - DL_Dq(4) - u4;
% EoM5 = DDL_DtDdq(5) - DL_Dq(5) - u5;
% EoM6 = DDL_DtDdq(6) - DL_Dq(6) - u6;
% EoM = [EoM1;EoM2;EoM3;EoM4;EoM5;EoM6];
% EoM = simplify(EoM);
% display(EoM(1));
Gq = subs(EoM, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, dth1, dth2, dth3, dth4, dth5, dth6, u1, u2, u3, u4, u5, u6], ...
               [    0,     0,     0,     0,     0,     0,    0,    0,    0,    0,    0,    0,  0,  0,  0,  0,  0,  0]);
% Gq = simplify(expand(Gq));

% Gq = subs(Gq, [th1 th2 th3 th4 th5 th6], [0 0 0 0 0 0]);

% double(Gq)

% display(Gq);

EoM_t = EoM - Gq;
Mq_t = subs(EoM_t, [dth1, dth2, dth3, dth4, dth5, dth6, u1, u2, u3, u4, u5, u6], [0,0,0,0,0,0, 0,0,0,0,0,0]);
Mq = [subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [1,0,0,0,0,0, 0,0,0,0,0,0]), ...
      subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [0,1,0,0,0,0, 0,0,0,0,0,0]), ...
      subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [0,0,1,0,0,0, 0,0,0,0,0,0]), ...
      subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [0,0,0,1,0,0, 0,0,0,0,0,0]), ...
      subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [0,0,0,0,1,0, 0,0,0,0,0,0]), ...
      subs(Mq_t, [ddth1, ddth2, ddth3, ddth4, ddth5, ddth6, u1, u2, u3, u4, u5, u6], [0,0,0,0,0,1, 0,0,0,0,0,0])];
% Mq(1,1) = expand(Mq(1, 1));
% simplify(Mq(1, 1));
% display(Mq);

EoM_tt = EoM_t - Mq_t; % C(q, dq)*dq = EoM - M(q)*ddq - Gq
Cqdq_dq = subs(EoM_tt, [u1, u2, u3, u4, u5, u6], [0,0,0,0,0,0]);
%Cqdq_dq = expand(Cqdq_dq);
%EoM_tt_ = vpa(EoM_tt, 7);
% 
% C111 = 0.5*(diff(D(1,1), th1) + diff(D(1,1), th1) - diff(D(1,1), th1));
% C211 = 0.5*(diff(D(1,1), th2) + diff(D(1,2), th1) - diff(D(2,1), th1));
% C311 = 0.5*(diff(D(1,1), th3) + diff(D(1,3), th1) - diff(D(3,1), th1));
% C411 = 0.5*(diff(D(1,1), th4) + diff(D(1,4), th1) - diff(D(4,1), th1));
% C511 = 0.5*(diff(D(1,1), th5) + diff(D(1,5), th1) - diff(D(5,1), th1));
% C611 = 0.5*(diff(D(1,1), th6) + diff(D(1,6), th1) - diff(D(6,1), th1));
% 
% C121 = 0.5*(diff(D(1,2), th1) + diff(D(1,1), th2) - diff(D(1,2), th1));
% C221 = 0.5*(diff(D(1,2), th2) + diff(D(1,2), th2) - diff(D(2,2), th1));
% C321 = 0.5*(diff(D(1,2), th3) + diff(D(1,3), th2) - diff(D(3,2), th1));
% C421 = 0.5*(diff(D(1,2), th4) + diff(D(1,4), th2) - diff(D(4,2), th1));
% C521 = 0.5*(diff(D(1,2), th5) + diff(D(1,5), th2) - diff(D(5,2), th1));
% C621 = 0.5*(diff(D(1,2), th6) + diff(D(1,6), th2) - diff(D(6,2), th1));
% 
% C131 = 0.5*(diff(D(1,3), th1) + diff(D(1,1), th3) - diff(D(1,3), th1));
% C231 = 0.5*(diff(D(1,3), th2) + diff(D(1,2), th3) - diff(D(2,3), th1));
% C331 = 0.5*(diff(D(1,3), th3) + diff(D(1,3), th3) - diff(D(3,3), th1));
% C431 = 0.5*(diff(D(1,3), th4) + diff(D(1,4), th3) - diff(D(4,3), th1));
% C531 = 0.5*(diff(D(1,3), th5) + diff(D(1,5), th3) - diff(D(5,3), th1));
% C631 = 0.5*(diff(D(1,3), th6) + diff(D(1,6), th3) - diff(D(6,3), th1));
% 
% C141 = 0.5*(diff(D(1,4), th1) + diff(D(1,1), th4) - diff(D(1,4), th1));
% C241 = 0.5*(diff(D(1,4), th2) + diff(D(1,2), th4) - diff(D(2,4), th1));
% C341 = 0.5*(diff(D(1,4), th3) + diff(D(1,3), th4) - diff(D(3,4), th1));
% C441 = 0.5*(diff(D(1,4), th4) + diff(D(1,4), th4) - diff(D(4,4), th1));
% C541 = 0.5*(diff(D(1,4), th5) + diff(D(1,5), th4) - diff(D(5,4), th1));
% C641 = 0.5*(diff(D(1,4), th6) + diff(D(1,6), th4) - diff(D(6,4), th1));
% 
% C151 = 0.5*(diff(D(1,5), th1) + diff(D(1,1), th5) - diff(D(1,5), th1));
% C251 = 0.5*(diff(D(1,5), th2) + diff(D(1,2), th5) - diff(D(2,5), th1));
% C351 = 0.5*(diff(D(1,5), th3) + diff(D(1,3), th5) - diff(D(3,5), th1));
% C451 = 0.5*(diff(D(1,5), th4) + diff(D(1,4), th5) - diff(D(4,5), th1));
% C551 = 0.5*(diff(D(1,5), th5) + diff(D(1,5), th5) - diff(D(5,5), th1));
% C651 = 0.5*(diff(D(1,5), th6) + diff(D(1,6), th5) - diff(D(6,5), th1));
% 
% C161 = 0.5*(diff(D(1,6), th1) + diff(D(1,1), th6) - diff(D(1,6), th1));
% C261 = 0.5*(diff(D(1,6), th2) + diff(D(1,2), th6) - diff(D(2,6), th1));
% C361 = 0.5*(diff(D(1,6), th3) + diff(D(1,3), th6) - diff(D(3,6), th1));
% C461 = 0.5*(diff(D(1,6), th4) + diff(D(1,4), th6) - diff(D(4,6), th1));
% C561 = 0.5*(diff(D(1,6), th5) + diff(D(1,5), th6) - diff(D(5,6), th1));
% C661 = 0.5*(diff(D(1,6), th6) + diff(D(1,6), th6) - diff(D(6,6), th1));
% 
% C1 = [C111, C211, C311, C411, C511, C611;
%       C121, C221, C321, C421, C521, C621;
%       C131, C231, C331, C431, C531, C631;
%       C141, C241, C341, C441, C541, C641;
%       C151, C251, C351, C451, C551, C651;
%       C161, C261, C361, C461, C561, C661] * [dth1;dth2;dth3;dth4;dth5;dth6];
% C1(1) = simplify(expand(C1(1)));

%matlabFunction(Mq,'File','Mq');
%matlabFunction(Gq,'File','Gq');

matlabFunction(Cqdq_dq(1),'File','Cqdq_dq1');
1
matlabFunction(Cqdq_dq(2),'File','Cqdq_dq2');
2
matlabFunction(Cqdq_dq(3),'File','Cqdq_dq3');
3
matlabFunction(Cqdq_dq(4),'File','Cqdq_dq4');
4
matlabFunction(Cqdq_dq(5),'File','Cqdq_dq5');
5
matlabFunction(Cqdq_dq(6),'File','Cqdq_dq6');
% for i = 1:6
%     for j =1:6
%         disp([i, j]);
%         Mq(i,j) = expand(Mq(i, j));
%         simplify(Mq(i, j));
%         fileID = fopen('tMq'+string(i)+string(j)+'.txt','w');
%         fprintf(fileID,'%s\n', char(vpa(Mq(i,j), 7)));
%         fclose(fileID); 
        
%         fileID = fopen('Cqdq'+string(i)+string(j) +'.txt','w');
%         fprintf(fileID,'%s\n',char(vpa(C1(1), 7)));
%         fclose(fileID);
        
%     end
% end
% EoM_tt(1) = simplify(expand(EoM_tt(1)));

% fileID = fopen('Cqdq11_.txt','w');
% fprintf(fileID,'%s\n',char(vpa(C1(1), 7)));
% fclose(fileID);
% fileID = fopen('Cqdq_dq_.txt','w');
% fprintf(fileID,'%s\n%s\n%s\n%s\n%s\n%s\n',char(vpa(EoM_tt(1), 7)), ...
%                                           char(vpa(EoM_tt(2), 7)), ...
%                                           char(vpa(EoM_tt(3), 7)), ...
%                                           char(vpa(EoM_tt(4), 7)), ...
%                                           char(vpa(EoM_tt(5), 7)), ...
%                                           char(vpa(EoM_tt(6), 7)));
% fclose(fileID);
