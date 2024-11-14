function [guards, new_modes] = solve_ivp_guard_funcs(guards_dict, mode, inputs, dt, parameters)
    % This function returns a single guard function handle and possible new modes.
    guard_funcs = {};
    new_modes = [];
    
    if isfield(guards_dict, mode)
        guard_keys = fieldnames(guards_dict.(mode));
        for i = 1:length(guard_keys)
            key = guard_keys{i};
            
            % Check if parameters are provided for guard function
            if isempty(parameters)
                % Create guard function without parameters
                guard_func = @(t, states) guards_dict.(mode).(key).g(t, states, inputs, dt);
            else
                % Create guard function with parameters
                guard_func = @(t, states) guards_dict.(mode).(key).g(t, states, inputs, dt, parameters);
            end
            
            guard_funcs{end+1} = guard_func;
            new_modes = [new_modes, key];
        end
    end

    % Output guards as a cell array of guard functions
    guards = guard_funcs;
end
