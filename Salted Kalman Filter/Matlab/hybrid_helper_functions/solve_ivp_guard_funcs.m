function [guards, new_modes] = solve_ivp_guard_funcs(guards_dict, mode, inputs, dt, parameters)
    % This function returns a single guard function handle and possible new modes.
    guard_funcs = {};
    new_modes = [];
    
    if isfield(guards_dict, mode)
        guard_keys = fieldnames(guards_dict.(mode));
        for i = 1:length(guard_keys)
            key = guard_keys{i};
            % Create each individual guard function
            guard_func = @(t, states) guards_dict.(mode).(key).g(t, states, inputs, dt, parameters);
            guard_funcs{end+1} = guard_func;
            new_modes = [new_modes, key];
        end
    end

    guards = guard_funcs;
    
    % Combine individual guard functions into a single function handle
    % guards = @(t, states) cellfun(@(g) g(t, states), guard_funcs);
end
