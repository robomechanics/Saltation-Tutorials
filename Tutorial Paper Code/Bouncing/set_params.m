function params = set_params()

% add more parameters as necessary, e.g. mass
params.g = 9.81; % gravity
% params.theta = atan2(-1, 1); % angle of slope
params.alpha = 0.8; %coeff of restitution

% consolidate parameters
params.parameters = [params.alpha, params.g];
end