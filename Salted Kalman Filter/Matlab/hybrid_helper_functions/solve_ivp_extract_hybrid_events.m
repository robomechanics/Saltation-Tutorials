function [hybrid_event_state, hybrid_event_time, new_mode] = solve_ivp_extract_hybrid_events(sol, possible_modes)
    % This function extracts the hybrid events from an ode45 solution.
    hybrid_event_state = [];
    hybrid_event_time = [];
    new_mode = [];
    
    for idx = 1:length(possible_modes)
        if ~isempty(sol.ie) && sol.ie == idx
            hybrid_event_state = sol.y(:, end);
            hybrid_event_time = sol.xe(end);
            new_mode = possible_modes(idx);
            return;
        end
    end
end