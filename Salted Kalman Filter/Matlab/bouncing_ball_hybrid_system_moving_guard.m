% Bouncing Ball System with Moving Guard
clear;
clc;
close all;

% Initialize symbolic variables for dynamics
syms q q_dot e g u dt t
states = [q; q_dot];
inputs = [u];
time = t;

% Define dynamics
fI = [q_dot; -g];
fJ = [q_dot; -g];

% Define measurements
yI = [q; q_dot];
yJ = [q; q_dot];

% Discretize the dynamics using Euler integration
fI_disc = states + fI * dt;
fJ_disc = states + fJ * dt;

% Take the Jacobian of dynamics with respect to states
AI_disc = jacobian(fI_disc, states);
AJ_disc = jacobian(fJ_disc, states);

% Measurement Jacobians
CI = jacobian(yI, states);
CJ = jacobian(yJ, states);

% Define resets
rIJ = [q; -e * q_dot];
rJI = [q; q_dot];

% Take the Jacobian of resets with respect to the states
RIJ = jacobian(rIJ, states);
RJI = jacobian(rJI, states);

% Define moving guard (sinusoidal paddle position)
gIJ = [q - 0.25 * sin(4 * pi * t)];
gJI = [q_dot];

% Take the Jacobian of guards with respect to states
GIJ = jacobian(gIJ, states);
GJI = jacobian(gJI, states);

% Take the Jacobian of guards with respect to time
GtIJ = jacobian(gIJ, time);
GtJI = jacobian(gJI, time);

% Convert symbolic expressions to MATLAB functions
fI_func = matlabFunction(fI, 'Vars', {states, inputs, dt, [e; g]});
AI_disc_func = matlabFunction(AI_disc, 'Vars', {states, inputs, dt, [e; g]});

fJ_func = matlabFunction(fJ, 'Vars', {states, inputs, dt, [e; g]});
AJ_disc_func = matlabFunction(AJ_disc, 'Vars', {states, inputs, dt, [e; g]});

yI_func = matlabFunction(yI, 'Vars', {states, [e; g]});
CI_func = matlabFunction(CI, 'Vars', {states, [e; g]});

yJ_func = matlabFunction(yJ, 'Vars', {states, [e; g]});
CJ_func = matlabFunction(CJ, 'Vars', {states, [e; g]});

rIJ_func = matlabFunction(rIJ, 'Vars', {states, inputs, dt, [e; g]});
RIJ_func = matlabFunction(RIJ, 'Vars', {states, inputs, dt, [e; g]});

rJI_func = matlabFunction(rJI, 'Vars', {states, inputs, dt, [e; g]});
RJI_func = matlabFunction(RJI, 'Vars', {states, inputs, dt, [e; g]});

gIJ_func = matlabFunction(gIJ, 'Vars', {time, states, inputs, dt, [e; g]});
GIJ_func = matlabFunction(GIJ, 'Vars', {states, inputs, dt, [e; g]});
GtIJ_func = matlabFunction(GtIJ, 'Vars', {time, states, inputs, dt, [e; g]});

gJI_func = matlabFunction(gJI, 'Vars', {time, states, inputs, dt, [e; g]});
GJI_func = matlabFunction(GJI, 'Vars', {states, inputs, dt, [e; g]});
GtJI_func = matlabFunction(GtJI, 'Vars', {time, states, inputs, dt, [e; g]});

% Define dictionaries
dynamics = struct( ...
    'I', struct('f_cont', fI_func, 'A_disc', AI_disc_func, 'y', yI_func, 'C', CI_func), ...
    'J', struct('f_cont', fJ_func, 'A_disc', AJ_disc_func, 'y', yJ_func, 'C', CJ_func) ...
);

resets = struct( ...
    'I', struct('J', struct('r', rIJ_func, 'R', RIJ_func)), ...
    'J', struct('I', struct('r', rJI_func, 'R', RJI_func)) ...
);

guards = struct( ...
    'I', struct('J', struct('g', gIJ_func, 'G', GIJ_func, 'Gt', GtIJ_func)), ...
    'J', struct('I', struct('g', gJI_func, 'G', GJI_func, 'Gt', GtJI_func)) ...
);

% Define noise matrices
n_states = 2;
W_global = 0.01 * eye(n_states);
V_global = 0.025 * eye(n_states);
noise_matrices = struct( ...
    'I', struct('W', W_global, 'V', V_global), ...
    'J', struct('W', W_global, 'V', V_global) ...
);

% Initial states and covariance
mean_init_state = [5; 0];
mean_init_cov = 0.1 * eye(n_states);
init_mode = 'I';

% Define time step and parameters
dt = 0.05;
parameters = [0.7; 9.8];  % [coefficient of restitution; gravity]

% Initialize filter (SKF)
skf = SKF(... 
          mean_init_state, ...
          init_mode, ...
          mean_init_cov, ...
          dt, ...
          noise_matrices, ...
          dynamics, ...
          resets, ...
          guards, ...
          parameters ...
          );

% Initialize simulator
actual_init_state = mvnrnd(mean_init_state, mean_init_cov)';
hybrid_simulator = HybridSimulator(... 
    actual_init_state, ... 
    init_mode, ... 
    dt, ... 
    noise_matrices, ... 
    dynamics, ... 
    resets, ... 
    guards, ... 
    parameters ...
    );

% Run simulation
n_simulate_timesteps = 100;
timesteps = 0:dt:(n_simulate_timesteps-1) * dt;
measurements = zeros(n_simulate_timesteps - 1, n_states);
actual_states = zeros(n_simulate_timesteps, n_states);
filtered_states = zeros(n_simulate_timesteps, n_states);
guard = 0.25 * sin(4 * pi * timesteps * dt);

actual_states(1, :) = hybrid_simulator.get_state()';
filtered_states(1, :) = mean_init_state';

% Loop over timesteps
for time_idx = 2:n_simulate_timesteps
    % Simulate actual system
    hybrid_simulator = hybrid_simulator.simulate_timestep(0, [0]);
    actual_states(time_idx, :) = hybrid_simulator.get_state()';
    measurements(time_idx - 1, :) = hybrid_simulator.get_measurement(true)';
    
    % Predict and update with SKF
    skf = skf.predict(timesteps(time_idx), [0]);
    [skf, filtered_states(time_idx, :), ~] = skf.update(timesteps(time_idx), [0], measurements(time_idx - 1, :)');
end

% Plot results
figure;
plot(actual_states(:, 1), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Actual states'); 
hold on;
plot(measurements(:, 1), 'r.', 'MarkerSize', 8, 'DisplayName', 'Measurements');
plot(filtered_states(:, 1), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered states'); 
plot(guard, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Guard'); 
legend('Location', 'best'); 
xlabel('Timestep');
ylabel('Position (y)');
title('1D Bouncing Ball Position with Moving Guard');
hold off;

figure;
plot(actual_states(:, 1), actual_states(:, 2), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Actual states');
hold on;
plot(measurements(:, 1), measurements(:, 2), 'r.', 'MarkerSize', 10, 'DisplayName', 'Measurements');
plot(filtered_states(:, 1), filtered_states(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered states');
legend('Location', 'best');
xlabel('y');
ylabel('$\dot{y}$', 'Interpreter', 'latex');
title('1D Bouncing Ball System Phase Plot with Moving Guard');
hold off;
