function x_plus = resets(t, x, u, domain, params)

    if domain == 1
        % switch from ascent to sliding
        theta = params.theta;
        x_plus = [x(1);
                  x(2);
                  0;
                  0];
    else
        error('Invalid Domain');
    end
end