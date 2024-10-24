% main: run simulations and stability calculations for each example system
% Diana Frias Franco
% modified from James Zhu
% Carnegie Mellon University Robomechanics Lab

clear;
close all;
clc;

rng(100)
% num of simulations to be run
num_sims = 25;

% x_0 should be a four by one: x = [q1; q2; q1_dot; q2_dot]
x = 0.25;
y = 10;
x_dot = 0.5;
y_dot = -0.125;
k = 0.25;
params = set_params();
theta = params.theta;

%create gaussian distribution of points
x0 = x;  % x-coordinate of the center
y0 = y;  % y-coordinate of the center
sigma = 0.12;  % Standard deviation of the distribution
% Generate Gaussian distribution of points
xyPoints = mvnrnd([x0, y0], sigma^2 * eye(2), num_sims);


% u stays empty vector for now, no input for this system
u = []; 

% length of simluation
t_f = 1.5;

figure;
y_ground = @(l) 1+tan(theta)*l;  % Equation for the slanted ground
fplot(y_ground, [0, x+2], 'r--', 'LineWidth', 2);
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');
title('Ball Trajectory');
grid on;
hold on;

for i=1:num_sims

    x_0 = [xyPoints(i, 1); xyPoints(i, 2); x_dot; y_dot];
    [state_vec,Xi_vec] = simulate_stick(x_0,u,t_f);


    x_trajectory = state_vec(:, 1);
    y_trajectory = state_vec(:, 2);
    % disp(-y_trajectory./x_trajectory - tan(theta) )
    plot(x_trajectory(1:2:end), y_trajectory(1:2:end), '.b', 'LineWidth', 2);
    plot(x_trajectory(1), y_trajectory(1), '*c')
    plot(x_trajectory(end), y_trajectory(end), '*c')
    hold on;
end

hold off;

% animate(state_vec, theta)
