function params = set_params()

% add more parameters as necessary, e.g. mass
params.g = 9.81;    % gravity
params.alpha = 0.7; % coeff of restitution

% consolidate parameters
params.parameters = [params.alpha, params.g];
end