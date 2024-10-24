function [value,isterminal,direction] = guards(t,x,u,domain,params)
    
    y = x(1);
    y_dot = x(2);
    
    g = [y;y_dot];
    
    % Check all contact conditions with this value
    value = g(domain); % The value that we want to be zero
    isterminal = ones(size(g(domain)));  % Halt integration and change contact modes
    direction = -ones(size(g(domain)));   % The zero can only be approached from this direction
end