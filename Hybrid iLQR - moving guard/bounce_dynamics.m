clear all
close all

% define variables
syms m g u theta theta_dot dt real

% define parameters
params = [m, g, l];

% define state and input
inputs = u;
states = [theta; theta_dot];

% define flow
flow = [theta_dot; (-m*g*l*sin(theta) + u)/(m*l^2)];

% set up discrete flow
flow_discrete = states + flow*dt;

% calculate A and B matrices
A_discrete = jacobian(flow_discrete, states);
B_discrete = jacobian(flow_discrete, inputs);


matlabFunction(flow_discrete,'File','calc_flow','Vars',[{states},{inputs},{dt},{params}]);
matlabFunction(A_discrete,'File','calc_A','Vars',[{states},{inputs},{dt},{params}]);
matlabFunction(B_discrete,'File','calc_B','Vars',[{states},{inputs},{dt},{params}]);

