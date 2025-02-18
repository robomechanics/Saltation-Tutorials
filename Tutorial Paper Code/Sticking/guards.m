function [value,isterminal,direction] = guards(t,x,u,domain,params)
    q1 = x(1);
    q2 = x(2);
    theta = params.theta;
    y_ground = @(l) 1+tan(-theta)*l;

    g = y_ground(q1);
    diff = abs(q2-g);
    % Check all contact conditions with this value
    if domain == 1
        value = (q2 - g); % stop the integration when this is zero
        isterminal = 1;  % Halt integration and change contact modes
        direction = -1;   % The zero can only be approached from this direction
    else % we shouldn't be able to switch out of this mode
        value = 10; % stop the integration when this is zero
        isterminal = 1;  % Halt integration and change contact modes
        direction = -1;   % The zero can only be approached from this direction
    end
end