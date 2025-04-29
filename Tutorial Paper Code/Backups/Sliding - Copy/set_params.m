function params = set_params()

% add more parameters as necessary, e.g. mass
params.g = 9.81; % gravity
params.theta = -atan2(-1, 1); % angle of slope

% consolidate parameters
params.parameters = [params.g params.theta];
end