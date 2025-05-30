clear all; close all;
syms m dadx dady g y y_dot uy dt coefficients e t real 
% Define the configuration and velocity variables
q_dot = y_dot;
q = y;
states = [q;q_dot];

% Define the ground to be at 0 height
a = -y;

% Jacobian of the ground constraint
A = jacobian(a,y);

% Defining the mass matrix
M = m*eye(1);

% Defining the block matrix when solving for constrained dynamics and reset
block_mat = [M,A';
             A,0];
block_mat_inv = inv(block_mat);

% Gravity
N = m*g;
% Thruster input
Y = uy;
inputs = Y;

q_ddot = inv(M)*(Y-N);

% Defining the dynamics of the system, since bouncing ball it is always
% flight
f1 = [q_dot;q_ddot];
f2 = [q_dot;q_ddot];

% Defining the linlearized dynamics of the system
A_lin1 = jacobian(f1,states);
B_lin1 = jacobian(f1,inputs);
A_lin2 = jacobian(f2,states);
B_lin2 = jacobian(f2,inputs);

%%
% Calculating reset maps
impact_vars = block_mat_inv*[M;-e*A]*q_dot; % Stores the post impact velocity and impulse
R1 = [y;impact_vars(1)];
R2 = states; % identity change

DR1 = jacobian(R1,states);
DR2 = jacobian(R2,states);

% Guards
x_p = 0.00005*sin(4*pi*t);

% x_p = 0;
G1 = -(y - x_p);
DG1 = jacobian(G1,states);
G2 = -y_dot;
DG2 = jacobian(G2,states);

% No time derivatives for the guards
DtG1 = -jacobian(G1, t);
DtG2 = sym(0);

parameters = [m,g,e];
%%
% New matlabfunction calls just to be consistent
matlabFunction(f2,'File','dynamics_helpers/flows/calc_f2','Vars',[{states},{inputs},{parameters}]);
matlabFunction(f1,'File','dynamics_helpers/flows/calc_f1','Vars',[{states},{inputs},{parameters}]);
matlabFunction(R1,'File','dynamics_helpers/resets/calc_r12','Vars',[{states},{inputs},{parameters}]);
matlabFunction(DR1,'File','dynamics_helpers/resets/calc_Dr12','Vars',[{states},{inputs},{parameters}]);
matlabFunction(R2,'File','dynamics_helpers/resets/calc_r21','Vars',[{states},{inputs},{parameters}]);
matlabFunction(DR2,'File','dynamics_helpers/resets/calc_Dr21','Vars',[{states},{inputs},{parameters}]);
matlabFunction(G1,'File','dynamics_helpers/guards/calc_g12','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(DG1,'File','dynamics_helpers/guards/calc_Dg12','Vars',[{states},{inputs},{parameters}]);
matlabFunction(DtG1,'File','dynamics_helpers/guards/calc_Dtg12','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(G2,'File','dynamics_helpers/guards/calc_g21','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(DG2,'File','dynamics_helpers/guards/calc_Dg21','Vars',[{states},{inputs},{parameters}]);
matlabFunction(DtG2,'File','dynamics_helpers/guards/calc_Dtg21','Vars',[{t}, {states},{inputs},{parameters}]);

matlabFunction(G1,'File','dynamics_helpers/guards/calc_a1','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(G2,'File','dynamics_helpers/guards/calc_a2','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(sym(A_lin1),'File','dynamics_helpers/lin_dyn/calc_A_lin1','Vars',[{states},{inputs},{parameters}]);
matlabFunction(sym(A_lin2),'File','dynamics_helpers/lin_dyn/calc_A_lin2','Vars',[{states},{inputs},{parameters}]);
matlabFunction(sym(B_lin1),'File','dynamics_helpers/lin_dyn/calc_B_lin1','Vars',[{states},{inputs},{parameters}]);
matlabFunction(sym(B_lin2),'File','dynamics_helpers/lin_dyn/calc_B_lin2','Vars',[{states},{inputs},{parameters}]);
%% Discrete
f1_disc = states+f1*dt;
f2_disc = states+f2*dt;
A_lin2_disc = jacobian(f2_disc,states);
A_lin1_disc = jacobian(f1_disc,states);
B_lin2_disc = jacobian(f2_disc,inputs);
B_lin1_disc = jacobian(f1_disc,inputs);

matlabFunction(f2_disc,'File','dynamics_helpers/disc_flow/calc_f2_disc','Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(f1_disc,'File','dynamics_helpers/disc_flow/calc_f1_disc','Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(A_lin1_disc,'File','dynamics_helpers/disc_flow/calc_A_lin1_disc','Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(A_lin2_disc,'File','dynamics_helpers/disc_flow/calc_A_lin2_disc','Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(B_lin1_disc,'File','dynamics_helpers/disc_flow/calc_B_lin1_disc','Vars',[{states},{inputs},{dt},{parameters}]);
matlabFunction(B_lin2_disc,'File','dynamics_helpers/disc_flow/calc_B_lin2_disc','Vars',[{states},{inputs},{dt},{parameters}]);

% Saltation matrix calculation
salt1 = simplify(DR1 + (calc_f2(R1,inputs,parameters)-DR1*f1)*DG1/(DtG1+DG1*f1));
salt2 = DR2; % Not bother calculating becuase it is identity
matlabFunction(salt1,'File','dynamics_helpers/salt/calc_salt12','Vars',[{t}, {states},{inputs},{parameters}]);
matlabFunction(salt2,'File','dynamics_helpers/salt/calc_salt21','Vars',[{t}, {states},{inputs},{parameters}]);