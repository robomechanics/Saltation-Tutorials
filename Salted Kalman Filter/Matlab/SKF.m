classdef SKF
    properties
        current_state        % Current state
        current_cov          % Current covariance matrix
        current_mode         % Current mode of the system
        dt                   % Time step
        noise_matrices_dict  % Noise matrices for each mode
        dynamics_dict        % Dynamics functions for each mode
        resets_dict          % Reset functions for each allowable transition
        guards_dict          % Guard functions for each allowable transition
        parameters           % System parameters
        n_states             % Number of states
    end
    
    methods
        function obj = SKF(init_state, init_mode, init_cov, dt, noise_matrices, dynamics, resets, guards, parameters)
            % Constructor
            obj.current_state = init_state;
            obj.current_cov = init_cov;
            obj.current_mode = init_mode;
            obj.dt = dt;
            obj.noise_matrices_dict = noise_matrices;
            obj.dynamics_dict = dynamics;
            obj.resets_dict = resets;
            obj.guards_dict = guards;
            obj.parameters = parameters;
            obj.n_states = length(init_state);
        end
        
        function [obj, current_state, current_cov] = predict(obj, current_time, inputs)
            % Perform the prediction (prior update) step
            end_time = current_time + obj.dt;
            
            % Set up dynamics function
            current_dynamics = solve_ivp_dynamics_func( ...
                obj.dynamics_dict, obj.current_mode, inputs, obj.dt, obj.parameters);
            % [current_guards, possible_modes] = solve_ivp_guard_funcs( ...
                % obj.guards_dict, obj.current_mode, inputs, obj.dt, obj.parameters);
            [current_guards, possible_modes] = obj.evaluateGuards(inputs);

            % Perform integration
            sol = ode45(current_dynamics, [current_time, end_time], obj.current_state, odeset('Events', current_guards));
            current_state = sol.y(:, end);
            
            % Check for hybrid events and apply reset if guard was hit
            [hybrid_event_state, hybrid_event_time, new_mode] = solve_ivp_extract_hybrid_events(sol, possible_modes);

            while ~isempty(new_mode)
                % Apply reset
                current_state = obj.resets_dict.(obj.current_mode).(new_mode).r( ...
                    hybrid_event_state, inputs, obj.dt, obj.parameters);
                
                % Apply covariance updates (dynamics and saltation matrix)
                dynamics_cov = obj.dynamics_dict.(obj.current_mode).A_disc( ...
                    obj.current_state, inputs, sol.x(end) - sol.x(1), obj.parameters);
                obj.current_cov = dynamics_cov * obj.current_cov * dynamics_cov' ...
                    + obj.noise_matrices_dict.(obj.current_mode).W;

                salt = compute_saltation_matrix( ...
                    hybrid_event_time, hybrid_event_state, inputs, obj.dt, obj.parameters, ...
                    obj.current_mode, new_mode, obj.dynamics_dict, obj.resets_dict, obj.guards_dict, current_state);
                obj.current_cov = salt * obj.current_cov * salt';
                
                % Update guard and re-simulate
                obj.current_mode = new_mode;
                current_dynamics = solve_ivp_dynamics_func( ...
                    obj.dynamics_dict, obj.current_mode, inputs, obj.dt, obj.parameters);
                % [current_guards, possible_modes] = solve_ivp_guard_funcs( ...
                    % obj.guards_dict, obj.current_mode, inputs, obj.dt, obj.parameters);
                [current_guards, possible_modes] = evaluateGuards(obj, inputs);
                sol = ode45(current_dynamics, [hybrid_event_time, end_time], current_state, odeset('Events', current_guards));
                [hybrid_event_state, hybrid_event_time, new_mode] = solve_ivp_extract_hybrid_events(sol, possible_modes);
            end

            % Apply final covariance update
            dynamics_cov = obj.dynamics_dict.(obj.current_mode).A_disc( ...
                obj.current_state, inputs, sol.x(end) - sol.x(1), obj.parameters);
            obj.current_cov = dynamics_cov * obj.current_cov * dynamics_cov' + obj.noise_matrices_dict.(obj.current_mode).W;
            
            % Update state and return
            obj.current_state = current_state;
            current_state = obj.current_state;
            current_cov = obj.current_cov;
        end
        
        function [obj, current_state, current_cov] = update(obj, current_time, current_input, measurement)
            % Perform the update (posterior update) step
            C = obj.dynamics_dict.(obj.current_mode).C(obj.current_state, obj.parameters);
            V = obj.noise_matrices_dict.(obj.current_mode).V;
            K = obj.current_cov * C' / (C * obj.current_cov * C' + V);

            % Measurement update
            measurement_est = obj.dynamics_dict.(obj.current_mode).y(obj.current_state, obj.parameters);
            residual = measurement - measurement_est;
            obj.current_state = obj.current_state + K * residual;
            obj.current_cov = (eye(obj.n_states) - K * C) * obj.current_cov;

            % Check guard conditions and apply reset if needed
            % [current_guards, possible_modes] = solve_ivp_guard_funcs(obj.guards_dict, obj.current_mode, current_input, obj.dt, obj.parameters);
            [current_guards, possible_modes] = evaluateGuards(obj, current_input);
            guard_names = fieldnames(obj.guards_dict.(obj.current_mode));
            for guard_idx = 1:length(current_guards)
                guard_name = guard_names{guard_idx};
                if current_guards(current_time, obj.current_state) < 0
                    % Apply reset due to guard condition
                    new_mode = possible_modes(guard_idx);
                    new_state = obj.resets_dict.(obj.current_mode).(new_mode).r(obj.current_state, current_input, obj.dt, obj.parameters);
                    
                    % Update covariance with saltation matrix
                    salt = compute_saltation_matrix( ...
                        current_time, obj.current_state, current_input, obj.dt, obj.parameters, ...
                        obj.current_mode, new_mode, obj.dynamics_dict, obj.resets_dict, obj.guards_dict, new_state);
                    obj.current_state = new_state;
                    obj.current_cov = salt * obj.current_cov * salt';
                    break;
                end
            end

            % Return updated state and covariance
            current_state = obj.current_state;
            current_cov = obj.current_cov;
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