function x_plus = resets(t, x, u, domain, params)

    if domain == 1
        % switch from ascent to sliding
        theta = params.theta;
        x_plus = [x(1);
                  x(2);
                  x(3)*cos(theta)*cos(theta) - x(4)*cos(theta)*sin(theta);
                  x(4)*sin(theta)*sin(theta) - x(3)*sin(theta)*cos(theta)];
    else
        error('Invalid Domain');
    end
end