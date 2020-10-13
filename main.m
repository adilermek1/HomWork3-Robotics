clc 
clear all
close all
d1 = 5;
a2 = 10; 
d3 = 20;
q_initial = [30*pi/180 60*pi/180];
T = FK_hw3(q_initial,d1,a2,d3)
draw_robot(q_initial,d1,a2,d3);

IK_hw3(T,d1,a2,d3)
q_initial
fprintf('Findind the Jacobian as geometric approach')
syms q1 q2 d1 a2 d3 
q_test1 = [q1 q2];
T_analitycal = FK_hw3(q_test1,d1,a2,d3)

T1 = T1_RRP(q_test1);
T2 = T2_RRP(q_test1,d1);
T3 = T3_RRP(q_test1,d1,a2);

z0 = [T1(1,3);T1(2,3);T1(3,3)];
z1 = [T2(1,3);T2(2,3);T2(3,3)];
z2 = [T3(1,3);T3(2,3);T3(3,3)];

Oc = [T_analitycal(1,4);T_analitycal(2,4);T_analitycal(3,4)];
O0 = [T1(1,4);T1(2,4);T1(3,4)];
O1 = [T2(1,4);T2(2,4);T2(3,4)];
O2 = [T3(1,4);T3(2,4);T3(3,4)];

J_geometric = [cross(z0,Oc-O0) cross(z1,Oc-O1) cross(z2,Oc-O2); z0 z1 z2]
d1 = 5;
a2 = 10; 
d3 = 20;
T1 = T1_RRP(q_initial);
T2 = T2_RRP(q_initial,d1);
T3 = T3_RRP(q_initial,d1,a2);

z0 = [T1(1,3);T1(2,3);T1(3,3)];
z1 = [T2(1,3);T2(2,3);T2(3,3)];
z2 = [T3(1,3);T3(2,3);T3(3,3)];

Oc = [T(1,4);T(2,4);T(3,4)];
O0 = [T1(1,4);T1(2,4);T1(3,4)];
O1 = [T2(1,4);T2(2,4);T2(3,4)];
O2 = [T3(1,4);T3(2,4);T3(3,4)];
J_geometric_initial = [cross(z0,Oc-O0) cross(z1,Oc-O1) cross(z2,Oc-O2); z0 z1 z2]

fprintf('Findind the Jacobian as classical approach(partial derivatives)')
syms q1 q2 d1 d3 a2 
q_test1 = [q1 q2];
H = FK_hw3(q_test1,d1,a2,d3);
R = H(1:3,1:3);
J1p = Rz(q1)*Tz(d1)*Rx(q2)*Ty(a2)*Ty(d3) ;
J1r = J1p*[R^-1 zeros(3,1);0 0 0 1];
J1 = [J1r(1,4);J1r(2,4);J1r(3,4);J1r(3,2);J1r(1,3);J1r(2,1)];

J2p = Rz(q1)*Tz(d1)*Rx(q2)*Ty(a2)*Ty(d3);
J2r = J2p*[R^-1 zeros(3,1);0 0 0 1];
J2 = [J2r(1,4);J2r(2,4);J2r(3,4);J2r(3,2);J2r(1,3);J2r(2,1)];

J3p = Rz(q1)*Tz(d1)*Rx(q2)*Ty(a2)*Ty(d3);
J3r = J3p*[R^-1 zeros(3,1);0 0 0 1];
J3 = [J3r(1,4);J3r(2,4);J3r(3,4);J3r(3,2);J3r(1,3);J3r(2,1)];

J = [J1 J2 J3];
d1 = 5;
a2 = 10; 
d3 = 20;
q_initial = [30*pi/180 60*pi/180];
H = FK_hw3(q_initial,d1,a2,d3);
R = H(1:3,1:3);
J1p = Rzd(q_initial(1))*Tz(d1)*Rx(q_initial(2))*Ty(a2)*Ty(d3) ;
J1r = J1p*[R^-1 zeros(3,1);0 0 0 1];
J1 = [J1r(1,4);J1r(2,4);J1r(3,4);J1r(3,2);J1r(1,3);J1r(2,1)];

J2p = Rz(q_initial(1))*Tz(d1)*Rxd(q_initial(2))*Ty(a2)*Ty(d3);
J2r = J2p*[R^-1 zeros(3,1);0 0 0 1];
J2 = [J2r(1,4);J2r(2,4);J2r(3,4);J2r(3,2);J2r(1,3);J2r(2,1)];

J3p = Rz(q_initial(1))*Tz(d1)*Rx(q_initial(2))*Ty(a2)*Ty(d3);
J3r = J3p*[R^-1 zeros(3,1);0 0 0 1];
J3 = [J3r(1,4);J3r(2,4);J3r(3,4);J3r(3,2);J3r(1,3);J3r(2,1)];

J = [J1,J2,J3]

fprintf('Computing the velocity when variables are changing')
d1 = 5;
a2 = 10;
syms t
d3 = sin(3*t);
q_test2 = [sin(t) cos(2*t)]
T1 = T1_RRP(q_test2);
T2 = T2_RRP(q_test2,d1);
T3 = T3_RRP(q_test2,d1,a2);

z0 = [T1(1,3);T1(2,3);T1(3,3)];
z1 = [T2(1,3);T2(2,3);T2(3,3)];
z2 = [T3(1,3);T3(2,3);T3(3,3)];

Oc = [T(1,4);T(2,4);T(3,4)];
O0 = [T1(1,4);T1(2,4);T1(3,4)];
O1 = [T2(1,4);T2(2,4);T2(3,4)];
O2 = [T3(1,4);T3(2,4);T3(3,4)];
J_changing = [cross(z0,Oc-O0) cross(z1,Oc-O1) cross(z2,Oc-O2); z0 z1 z2]






