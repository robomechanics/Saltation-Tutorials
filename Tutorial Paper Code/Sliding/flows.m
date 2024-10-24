function dxVec = flows(t, x, u, domain, params)

    g = params.g;
    theta = params.theta;
        
    if domain == 1
        % we are in the descent domain
        dxVec = zeros(4,1);
        
        dxVec(1,1) = x(3);  % q1_dot
        dxVec(2,1) = x(4);  % q2_dot
        dxVec(3,1) = 0;     % q1_double_dot = 0 b/c no control inputs
        dxVec(4,1) = -g;    % q2_double_dot = acc due to gravity

    elseif domain == 2
        % we are in the sliding domain
        dxVec = zeros(4,1);
        
        dxVec(1,1) = x(3);  % q1_dot
        dxVec(2,1) = x(4);  % q2_dot
        dxVec(3,1) = g*sin(theta)*cos(theta);     % q1_double_dot = 0 b/c no control inputs
        dxVec(4,1) = -g*sin(theta)*sin(theta);    % q2_double_dot = acc due to gravity
        
    else
        error('Invalid Domain');
    end
end