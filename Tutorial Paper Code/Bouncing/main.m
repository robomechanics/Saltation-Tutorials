% main: run simulations and stability calculations for each example system
% Diana Frias Franco
% modified from James Zhu
% Carnegie Mellon University Robomechanics Lab

clear;
close all;
clc;

navy = [35, 110, 150]/255;
pink = [255 89 143]/255;
teal = [21, 178, 211]/255;
yellow = [255, 215, 0]/255;
orange = [243, 135, 47]/255;


rng(100)
% num of simulations to be run
num_sims = 1; %25;

% x_0 should be a two by one: x = [q1; q1_dot]
x = 1;
x_dot = 0;
k = 0.25;
params = set_params();
% theta = params.theta;


% u stays empty vector for now, no input for this system
u = []; 

% length of simluation
t_f = 4.2;

figure;
y_ground = @(l) 0;  % Equation for the slanted ground
fplot(y_ground, [0, x+20], 'k--', 'LineWidth', 2);
ylabel('Horizontal Distance (m)');
xlabel('time(s)');
title('Ball Trajectory');
grid on;
hold on;
xlim([3 4.5])

for i=1:num_sims
    dx = 0; %rand(1, 1)*2 - 1;

    x_0 = [x + dx*k; x_dot];

    [state_vec,Xi_vec, time_vec, cp, ct] = simulate_bounce(x_0,u,t_f);

    y_trajectory = state_vec(:, 1);
    x_trajectory = time_vec; %zeros(size(y_trajectory, 1), 1); %state_vec(:, 2);
    % disp(-y_trajectory./x_trajectory - tan(theta) )
    plot(x_trajectory(1:2:end), y_trajectory(1:2:end), '.b', 'LineWidth', 2);
    plot(x_trajectory(1), y_trajectory(1), '*c')
    plot(x_trajectory(end), y_trajectory(end), '*c')
    plot(ct, cp, '--r')
    

    hold on;
end

hold off;
% f = fit(ct',cp','exp1')
% animate(state_vec)
% animate(state_vec, time_vec)