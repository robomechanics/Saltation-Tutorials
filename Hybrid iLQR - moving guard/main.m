%% startup
close all;
clear all;
clc 

%% define system dynamics
bouncing_dynamics();

%% define parameters
m = 1; % pendulum mass
g = 9.8; % gravity
l = 1; % pendulum length
params = [m, g, l];

%% discretize time
dt = 0.005; % will prob need to adjust later
t_start = 0;
t_end = 5; % may also need to adjust
time_vector = t_start:dt:t_end;

%% initial state + goal state
x0 = [0; 0]; % 0 position and 0 velocity
x_goal = [pi; 0]; % want pedulum to end up inverted

%% initial input guess
u0 = zeros(size(time_vector, 2), 1); % all zeros to start off with

%% define cost parameters
Q_k = zeros(2, 2); % size should match # states
R_k = 0.001*eye(1); % size should match # inputs, may also need to be tuned
Q_T = 100*eye(2); % terminal cost, may also need tuning

%% define iLQR parameters
max_iter = 100;
tolerance = 1e-6; % difference in expected cost reduction

%% run iLQR algorithm
ilqr_sys = ilqr(params, dt, t_start, t_end, x0, x_goal, u0, Q_k, R_k, Q_T, max_iter, @calc_A, @calc_B, @calc_flow);
[states, inputs, k_feedforward, K_feedback, final_cost] = ilqr_sys.mainSolve();

%% plot results
figure(1);
h1 = plot(states(:,1),states(:,2),'k');
hold on
h2 = scatter(x_goal(1),x_goal(2),200, 'MarkerFaceColor', [247, 114, 174]/255, 'MarkerEdgeColor', [0, 0, 0]);
hold on
h3 = scatter(states(1,1),states(1,2), 200, 'MarkerFaceColor', [129, 137, 247]/255, 'MarkerEdgeColor', [0, 0, 0]);
legend([h3, h1,h2],"Initial State", "iLQR Trajectory","Goal State");
xlabel('$$\theta$$', 'Interpreter', 'latex');
ylabel('$$\dot{\theta}$$', 'Interpreter', 'latex');
title("Pendulum Trajectory")
%% animate results
% animate(states, dt)
% 