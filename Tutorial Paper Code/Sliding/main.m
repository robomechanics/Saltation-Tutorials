% main: run simulations and stability calculations for each example system
% Diana Frias Franco
% modified from James Zhu
% Carnegie Mellon University Robomechanics Lab

clear;
% close all;
clc;

rng(10)
% num of simulations to be run
num_sims = 50;

% x_0 should be a four by one: x = [q1; q2; q1_dot; q2_dot]
x = 0.25;
y = 5;
x_dot = 0.5;
y_dot = -0.125;
k = 0.25;
params = set_params();
theta = params.theta;


% u stays empty vector for now, no input for this system
u = []; 

% length of simluation
navy = [35, 110, 150]/255;
pink = [255 89 143]/255;
teal = [21, 178, 211]/255;
yellow = [255, 215, 0]/255;
orange = [243, 135, 47]/255;

t_f = 1.1;
figure;
y_ground = @(l) 1+tan(-theta)*l;  % Equation for the slanted ground
fplot(y_ground, [-2, x+2], 'LineStyle', '-', 'Color', 'k', 'LineWidth', 2);
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');
xlim([-0.5 2.5])
title('Ball Trajectory');
grid on;
hold on;

start_states = [];
end_states = [];
for i=1:num_sims
    dx = rand(1, 1)*2 - 1;
    dy = rand(1, 1)*2 - 1;

    x_0 = [x + dx*k; y + dy*k*2; x_dot + dx*k/5; y_dot + dy*k/5];
    [state_vec, Xi_vec, t_event, t_vec] = simulate_slide(x_0,u,t_f);

    
    x_trajectory = state_vec(:, 1);
    y_trajectory = state_vec(:, 2);
    % disp(-y_trajectory./x_trajectory - tan(theta) )
%     plot(x_trajectory(1:2:end), y_trajectory(1:2:end), '.b', 'LineWidth', 2);

    start_states = [start_states; state_vec(1, :)];
    end_states = [end_states; state_vec(end, :)];

    
    hold on;
end

hold off;

% add the mean path
hold on;


x_0 = [x; y; x_dot; y_dot];
[state_vec, Xi_1_vec, Xi_2_vec, t_event, t_vec] = simulate_slide(x_0,u,t_f);
x_trajectory = state_vec(:, 1);
y_trajectory = state_vec(:, 2);

start_states = [start_states; state_vec(1, :)];
end_states = [end_states; state_vec(end, :)];


transparency = 0.5;
plot(x_trajectory(1:2:end), y_trajectory(1:2:end), 'LineStyle', '--', 'Color', teal, 'LineWidth', 1);
s1 = scatter(start_states(:, 1), start_states(:, 2), 'MarkerFaceColor',navy,'MarkerEdgeColor', 'k');
s2 = scatter(end_states(:, 1), end_states(:, 2), 'MarkerFaceColor',navy,'MarkerEdgeColor', 'k'); 

s1.MarkerFaceAlpha = transparency;
% s1.MarkerEdgeAlpha = transparency;
s2.MarkerFaceAlpha = transparency;
% s2.MarkerEdgeAlpha = transparency;
% legend('','Average trajectory', '', '')
hold off;



% now add the covariances to the plots
% to do this, we're just going to step forward through the mean trajectory
% and when a transition happens, we'll apply the saltation matrix
start_cov = cov(start_states);
current_cov = start_cov;
naive_cov = current_cov;
event = 0; % for debugging
prev_t = t_vec(1);
for ii = 1:numel(t_vec)
    current_time = t_vec(ii);
    curr_t = current_time;
    dt = curr_t - prev_t;
    A = [1 0 dt 0;
        0 1 0 dt;
        0 0 1 0;
        0 0 0 1];
    current_cov = A*current_cov*A';
    naive_cov = A*naive_cov*A';
    if(t_event(1) == current_time)
        if event == 0
%             disp('event');
%             disp(current_time)
            salt = Xi_1_vec{1};
            jac = Xi_2_vec{1};
            current_cov = salt*current_cov*salt';
%             naive_cov = jac*current_cov*jac';
            event = 1;
        end
    else
        
    end
    prev_t = curr_t;
end
end_cov = cov(end_states);

hold on
% error_ellipse(start_cov([1,2],[1,2]), state_vec(end, 1:2))
e1 = error_ellipse(naive_cov([1,2],[1,2]), state_vec(end, 1:2), 'conf', 0.95);
e2 = error_ellipse(current_cov([1,2],[1,2])' + eye(2)*0.0000001, state_vec(end, 1:2), 'conf', 0.95);
% error_ellipse(end_cov([1,2],[1,2]), state_vec(end, 1:2))
e1.LineStyle = '--';
e1.Color = orange;
e1.LineWidth = 2;


e2.LineStyle = '-';
e2.Color = yellow; %[248, 252, 121]/255;
e2.LineWidth = 2;


legend('','Average trajectory', '', '', '$D_xR$ Prediction', '$\Xi$ Prediction', 'Interpreter', 'latex')
hold off

% animate(state_vec, t_vec, theta)

% some debugging stuff
[l, ll] = eig(salt([1,2],[1,2]));