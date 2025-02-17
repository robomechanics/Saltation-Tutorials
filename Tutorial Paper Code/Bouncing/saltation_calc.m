clear all; close all;
clc

syms y y_dot x_P x_P_dot x_P_ddot alpha g t real 
parameters = [alpha g];

states = [y; y_dot];

% mode 1 is descent. mode 2 is ascent
% This is a reset map
states_2_to_1 = [y; y_dot];
states_1_to_2 = [y; - alpha*y_dot];

R12 = states_1_to_2;
DR12 = jacobian(R12,states);
R21 = states_2_to_1;
DR21 = jacobian(R21,states);
% no time variance
DtR12 = sym(zeros(2,1));
DtR21 = sym(zeros(2,1));

f1 = [y_dot; -g];
f2 = [-alpha*y_dot; -g];

g1 = y;
Dg1 = jacobian(g1,states);
g2 = y_dot;
Dg2 = jacobian(g2,states);
Dtg1 = 0;
Dtg2 = 0;

Xi_1 = simplify(DR12+(f2 - DR12*f1 - DtR12)*Dg1/(Dtg1 + Dg1*f1));
Xi_2 = simplify(DR21+(f1 - DR21*f1 - DtR21)*Dg2/(Dtg2 + Dg2*f2));

matlabFunction(Xi_1,'File','calc_Xi_1','Vars',[{states},{parameters}]);
matlabFunction(Xi_2,'File','calc_Xi_2','Vars',[{states},{parameters}]);