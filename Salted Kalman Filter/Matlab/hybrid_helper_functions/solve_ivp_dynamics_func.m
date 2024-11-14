function dynamics_func = solve_ivp_dynamics_func(dynamics_dict, mode, inputs, dt, parameters, process_gaussian_noise)
    % This function creates a function handle compatible with ode45, adding process noise if provided.
    if nargin > 5 && ~isempty(process_gaussian_noise)
        process_noise = mvnrnd(process_gaussian_noise.mean, process_gaussian_noise.cov)';
        dynamics_func = @(t, states) dynamics_dict.(mode).f_cont(states, inputs, dt, parameters) + process_noise;
    else
        dynamics_func = @(t, states) dynamics_dict.(mode).f_cont(states, inputs, dt, parameters);
    end
end

