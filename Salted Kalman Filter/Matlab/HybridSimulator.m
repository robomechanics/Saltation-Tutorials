classdef HybridSimulator
    properties
        current_state    % Current state of the system
        current_mode     % Current
        dt               % Time step
        dynamics_dict    % Dynamics functions for each mode
        resets_dict      % Reset functions for transitions
        guards_dict      % Guard functions for transitions
        parameters       % System parameters
        noise_matrices   % Noise matrices for each mode
        n_states         % Number of states
    end

    methods
        function obj = HybridSimulator(init_state, init_mode, dt, noise_matrices, dynamics, resets, guards, parameters)
            % Constructor
            obj.current_state = init_state;
            obj.current_mode = init_mode;
            obj.dt = dt;
            obj.dynamics_dict = dynamics;
            obj.resets_dict = resets;
            obj.guards_dict = guards;
            obj.parameters = parameters;
            obj.noise_matrices = noise_matrices;
            obj.n_states = length(init_state);
        end

        function obj = simulate_timestep(obj, current_time, inputs)
            % Simulate for one time step
            end_time = current_time + obj.dt;

            % Define process noise for the current mode
            process_noise = mvnrnd(zeros(obj.n_states, 1), obj.noise_matrices.(obj.current_mode).W)';

            % Get dynamics function for the current mode
            current_dynamics = solve_ivp_dynamics_func(obj.dynamics_dict, obj.current_mode, inputs, obj.dt, obj.parameters, struct('mean', zeros(obj.n_states, 1), 'cov', obj.noise_matrices.(obj.current_mode).W));
            % current_dynamics = @(t, x) obj.dynamics_dict{obj.current_mode}(x, inputs, obj.parameters) + process_noise;
            [current_guards, possible_modes] = obj.evaluateGuards(inputs);

            % Run the integration for the current step
            options = odeset('Events', current_guards);
            [time, sol, ~, ~, ie] = ode45(current_dynamics, [current_time, end_time], obj.current_state, options);

            % Update state at the end of integration
            obj.current_state = sol(end, :)';

            % If guard was hit, handle the event
            if ~isempty(ie)
                [obj.current_state, obj.current_mode] = obj.applyReset(sol(end, :)', inputs, possible_modes(ie));
            end
        end

        function measurement = get_measurement(obj, measurement_noise_flag)
            % Get measurement with or without noise
            if isempty(obj.parameters)
                % Call the measurement function without parameters
                measurement = obj.dynamics_dict.(obj.current_mode).y(obj.current_state);
            else
                % Call the measurement function with parameters
                measurement = obj.dynamics_dict.(obj.current_mode).y(obj.current_state, obj.parameters);
            end

            % Add noise if specified
            if measurement_noise_flag
                noise = mvnrnd(zeros(size(measurement)), obj.noise_matrices.(obj.current_mode).V);
                measurement = measurement + noise';
            end
        end

        function state = get_state(obj)
            % Get a copy of the current state
            state = obj.current_state;
        end
    end

    methods (Access = private)
        function [guards, possible_modes] = evaluateGuards(obj, inputs)
            % Generate guard functions and possible modes for the current mode
            [guard_funcs, possible_modes] = solve_ivp_guard_funcs(obj.guards_dict, obj.current_mode, inputs, obj.dt, obj.parameters);

            % Combine individual guard functions into a single function handle
            % compatible with ode45's event detection requirements
            guards = @(t, states) obj.evaluateGuardsForODE(guard_funcs, t, states);
        end

        function [value, isterminal, direction] = evaluateGuardsForODE(obj, guard_funcs, t, states)
            % Evaluate each guard function and return in a format compatible with ode45
            value = cellfun(@(g) g(t, states), guard_funcs); % Evaluate all guard functions
            isterminal = ones(size(value));    % Stop integration if any guard is triggered
            direction = -ones(size(value));    % Detect zero crossings in the decreasing direction
        end

        function [new_state, new_mode] = applyReset(obj, impact_state, inputs, new_mode)
            % Apply the reset function and return new state and mode
            reset_func = obj.resets_dict.(obj.current_mode).(new_mode).r;
            new_state = reset_func(impact_state, inputs, obj.dt, obj.parameters);
        end
    end
end

