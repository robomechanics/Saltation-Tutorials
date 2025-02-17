close all;
clear all;
clc

% Compute symbolic dynamics and create functions
bouncing_dynamics();

% Initialize timings
dt = 0.004;
start_time = 0;
end_time = 1;
time_span = start_time:dt:end_time;

% Set desired state
n_states = 2;
n_inputs = 1;
init_state = [4;0]; 
init_mode = 1;
target_state = [1;0];
x_goal = target_state;

% Initial guess of zeros, but you can change it to any guess
initial_guess = -100.0*ones(size(time_span,2),1);

% Define weighting matrices
Q_k = zeros(n_states,n_states); % zero weight to penalties along a strajectory since we are finding a trajectory
R_k = 1*(5*10^-7 / dt)*eye(n_inputs);

% Set the terminal cost
Q_T = 100*eye(n_states);

% Set the physical parameters of the system
mass = 1;
gravity = 9.8;
coefficient_restitution = 0.7;
parameters = [mass,gravity,coefficient_restitution];

% Specify max number of iterations
n_iterations = 50;

% Specify optimality condition
min_reduction = 0.05; %1e-2;

% Define optimization struct
optimization_struct.init_state = init_state;
optimization_struct.init_mode = init_mode;
optimization_struct.target_state = target_state;
optimization_struct.initial_guess = initial_guess;
optimization_struct.dt = dt;
optimization_struct.start_time = start_time;
optimization_struct.end_time = end_time;
optimization_struct.n_iterations = n_iterations;
optimization_struct.min_reduction = min_reduction;
optimization_struct.Q_T = Q_T;
optimization_struct.Q_k = Q_k;
optimization_struct.R_k = R_k;

% Construct ilqr object
A_disc = {@calc_A_lin1_disc,@calc_A_lin2_disc};
B_disc = {@calc_B_lin1_disc,@calc_B_lin2_disc};
f = {@calc_f1,@calc_f2};
resets = {@calc_r12,@calc_r21};
guards = {@calc_g12,@calc_g21};
guard_jacobians = {@calc_Dg12,@calc_Dg21};
salts = {@calc_salt12,@calc_salt21};

% Define dynamics struct
dynamics_struct.A_disc = A_disc;
dynamics_struct.B_disc = B_disc;
dynamics_struct.f = f;
dynamics_struct.resets = resets;
dynamics_struct.guards = guards;
dynamics_struct.guard_jacobians = guard_jacobians;
dynamics_struct.salts = salts;
dynamics_struct.parameters = parameters;

ilqr_ = h_ilqr(optimization_struct,dynamics_struct);

% Solve
[states,inputs,modes,trajectory_struct,k_feedforward,K_feedback,final_cost,expected_reduction, rollout_states, rollout_inputs, prev_traj, iter] = ilqr_.solve();
save('single-bounce-trajectory.mat','states','inputs','modes','trajectory_struct','dt','parameters','k_feedforward','K_feedback'); % Save trajectory to track later

%% plot results
figure(1);
h1 = plot(states(:,1),states(:,2),'k');
hold on
h4 = plot(rollout_states(:,1),rollout_states(:,2),'r--');
hold on
h2 = scatter(x_goal(1),x_goal(2),200, 'MarkerFaceColor', [247, 114, 174]/255, 'MarkerEdgeColor', [0, 0, 0]);
hold on
h3 = scatter(states(1,1),states(1,2), 200, 'MarkerFaceColor', [129, 137, 247]/255, 'MarkerEdgeColor', [0, 0, 0]);
legend([h3, h4, h1,h2],"Initial State", "Rollout Trajectory", "h-iLQR Trajectory","Goal State");
xlabel('$$y$$', 'Interpreter', 'latex');
ylabel('$$\dot{y}$$', 'Interpreter', 'latex');
title("Bouncing Ball Trajectory")
hold off

% pos vs time plot
diff = -states(:,1) + prev_traj(:, 1);
figure(2);
p1 = plot(states(:,1), 'r--');
hold on
p2 = plot(rollout_states(:,1), 'k');
hold on
p3 = scatter(size(states, 1), x_goal(1),100, 'MarkerFaceColor', [247, 114, 174]/255, 'MarkerEdgeColor', [0, 0, 0]);
hold on
y_ground = @(l) 0.00005*sin(4*pi*l*dt);  % Equation for the ground
p4 = fplot(y_ground, 'k--', 'LineWidth', 2);
xlabel("Timestep")
ylabel("y")
legend([p1, p2, p3, p4], "h-iLQR", "Rollout", "Goal State", "Guard")
title("Iteration: " + num2str(iter) + ", Position")

hold off;
xlim([0, 252])

