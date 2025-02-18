function dxVec = flows(t, x, u, domain, params)
    g = params.g;
        
    if domain == 1
        dxVec = zeros(2,1);
    
        dxVec(1,1) = x(2);
        dxVec(2,1) = -g;
    elseif domain == 2
        dxVec = zeros(2,1);
    
        dxVec(1,1) = x(2);
        dxVec(2,1) = -g;
    else
        error('Invalid Domain');
    end
end