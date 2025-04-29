function x_plus = resets(t, x, u, domain, params)

if domain == 1
    alpha = params.alpha;
    x_plus = [x(1);-alpha*x(2)];
elseif domain == 2
    x_plus = x;
else
    error('Invalid Domain');
end