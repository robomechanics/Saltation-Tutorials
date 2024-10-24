clear all; close all;
clc

syms x y x_dot y_dot theta g t real 
parameters = [g theta];
%%
states = [x; y; x_dot; y_dot];

% mode 1 is descent. mode 2 is sliding
% This is a reset map
states_1_to_2 = [x;
                 y;
                 0;
                 0];

R12 = states_1_to_2;
DR12 = jacobian(R12,states);

% no time variance
DtR12 = sym(zeros(4,1));

f1 = [x_dot; y_dot; 0; -g];
f2 =   [0;
        0;
        0;
        0];

g1 = x*sin(theta) + y*cos(theta);
Dg1 = jacobian(g1,states);

Dtg1 = 0;


Xi_1 = simplify(DR12+(f2 - DR12*f1 - DtR12)*Dg1/(Dtg1 + Dg1*f1));
Xi_2 = simplify(DR12);

matlabFunction(Xi_1,'File','calc_Xi_1','Vars',[{states},{parameters}]);
matlabFunction(Xi_2,'File','calc_Xi_2','Vars',[{states},{parameters}]);