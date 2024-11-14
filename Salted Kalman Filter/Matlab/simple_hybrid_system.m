clear; clc; close all;

syms x1 x2 u dt t params_placeholder
states = [x1; x2];
inputs = [u];
n_states = 2;
time = t;

% Define dynamics
fI = sym([1; -1]);
fJ = sym([1; 1]);

% Define measurements
yI = [x1; x2];
yJ = [x1; x2];

% Discretize dynamics using Euler integration
fI_disc = states + fI * dt;
fJ_disc = states + fJ * dt;

% Take the Jacobian with respect to states
AI_disc = jacobian(fI_disc, states);
AJ_disc = jacobian(fJ_disc, states);

% Measurement Jacobians
CI = jacobian(yI, states);
CJ = jacobian(yJ, states);

% Define resets
rIJ = [x1; x2];  % No change in the reset function in this case
rJI = [x1; x2];   

% Take the Jacobian of resets with respect to the states
RIJ = jacobian(rIJ, states);
RJI = jacobian(rIJ, states);

% Define guard functions
gIJ = [-x1];  % Switch when x1 < 0
gJI = [x1 - 10];       % No valid transition from J to I by default

% Take the Jacobian of guards with respect to states
GIJ = jacobian(gIJ, states);
GJI = jacobian(gJI, states); % Zero matrix as no guard is active for J to I

% Take the Jacobian of guards with respect to time
GtIJ = jacobian(gIJ, time);
GtJI = jacobian(gJI, time);

% Convert symbolic expressions to MATLAB functions
fI_func = matlabFunction(fI, 'Vars', {states, inputs, dt, params_placeholder});
AI_disc_func = matlabFunction(AI_disc, 'Vars', {states, inputs, dt, params_placeholder});

fJ_func = matlabFunction(fJ, 'Vars', {states, inputs, dt, params_placeholder});
AJ_disc_func = matlabFunction(AJ_disc, 'Vars', {states, inputs, dt, params_placeholder});

yI_func = matlabFunction(yI, 'Vars', {states, params_placeholder});
CI_func = matlabFunction(CI, 'Vars', {states, params_placeholder});

yJ_func = matlabFunction(yJ, 'Vars', {states, inputs, dt, params_placeholder});
CJ_func = matlabFunction(CJ, 'Vars', {states, inputs, dt, params_placeholder});

rIJ_func = matlabFunction(rIJ, 'Vars', {states, inputs, dt, params_placeholder});
RIJ_func = matlabFunction(RIJ, 'Vars', {states, inputs, dt, params_placeholder});
rJI_func = matlabFunction(rJI, 'Vars', {states, inputs, dt, params_placeholder});
RJI_func = matlabFunction(RJI, 'Vars', {states, inputs, dt, params_placeholder});

gIJ_func = matlabFunction(gIJ, 'Vars', {time, states, inputs, dt, params_placeholder});
GIJ_func = matlabFunction(GIJ, 'Vars', {states, inputs, dt, params_placeholder});
GtIJ_func = matlabFunction(GtIJ, 'Vars', {time, states, inputs, dt, params_placeholder});

gJI_func = matlabFunction(gJI, 'Vars', {time, states, inputs, dt, params_placeholder});
GJI_func = matlabFunction(GJI, 'Vars', {states, inputs, dt, params_placeholder});
GtJI_func = matlabFunction(GtJI, 'Vars', {time, states, inputs, dt, params_placeholder});

% Define the dynamics, resets, and guards in struct format
dynamics = struct( ...
    'I', struct('f_cont', fI_func, 'A_disc', AI_disc_func, 'y', yI_func, 'C', CI_func), ...
    'J', struct('f_cont', fJ_func, 'A_disc', AJ_disc_func, 'y', yJ_func, 'C', CJ_func) ...
);

resets = struct('I', struct('J', struct('r', rIJ_func, 'R', RIJ_func)), ...
                'J', struct('I', struct('r', rJI_func, 'R', RJI_func)));

guards = struct('I', struct('J', struct('g', gIJ_func, 'G', GIJ_func, 'Gt', GtIJ_func)), ...
                'J', struct('I', struct('g', gJI_func, 'G', GJI_func, 'Gt', GtJI_func)));

% Define global noise matrices for process and measurement noise
n_states = 2;
W_global = 0.01 * eye(n_states);  % Process noise covariance
V_global = 0.025 * eye(n_states);  % Measurement noise covariance
noise_matrices = struct( ...
    'I', struct('W', W_global, 'V', V_global), ...
    'J', struct('W', W_global, 'V', V_global) ...
);

% Initial states and covariance
mean_init_state = [-2.5; 0];
mean_init_cov = 0.1 * eye(n_states);
init_mode = 'I';

% Define time step and parameters
dt = 0.1;
parameters = 0;

% Initialize filter (SKF)
skf = SKF(mean_init_state, init_mode, mean_init_cov, dt, noise_matrices, dynamics, resets, guards, parameters);
% skf = SKF(mean_init_state, init_mode, mean_init_cov, dt, [], dynamics, resets, guards, parameters);

% Initialize simulator
actual_init_state = mvnrnd(mean_init_state, mean_init_cov)';
hybrid_simulator = HybridSimulator(actual_init_state, init_mode, dt, noise_matrices, dynamics, resets, guards, parameters);
% hybrid_simulator = HybridSimulator(actual_init_state, init_mode, dt, [], dynamics, resets, guards, parameters);

n_simulate_timesteps = 50;
timesteps = 0:dt:(n_simulate_timesteps-1) * dt;
measurements = zeros(n_simulate_timesteps - 1, n_states);
actual_states = zeros(n_simulate_timesteps, n_states);
filtered_states = zeros(n_simulate_timesteps, n_states);

actual_states(1, :) = hybrid_simulator.get_state()';
filtered_states(1, :) = mean_init_state';

% Simulation loop
for time_idx = 2:n_simulate_timesteps
    % Simulate actual system
    hybrid_simulator = hybrid_simulator.simulate_timestep(0, [0]);
    actual_states(time_idx, :) = hybrid_simulator.get_state()';
    measurements(time_idx - 1, :) = hybrid_simulator.get_measurement(true)';

    % Predict and update with SKF
    skf = skf.predict(timesteps(time_idx), [0]);
    [skf, filtered_states(time_idx, :), ~] = skf.update(timesteps(time_idx), [0], measurements(time_idx - 1, :)');
end

% Plot results1
figure;
plot(actual_states(:, 1), actual_states(:, 2), 'k-', 'LineWidth', 2, 'DisplayName', 'Actual states');
hold on;
plot(measurements(:, 1), measurements(:, 2), 'r.', 'MarkerSize', 10, 'DisplayName', 'Measurements');
plot(filtered_states(:, 1), filtered_states(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered states');
legend;
xlabel('y');
ylabel('$\dot{y}$', 'Interpreter', 'latex');
title('1D Bouncing Ball System');
hold off;


