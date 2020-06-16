function dx = vehicle_dynamics(x,u,p)

    %% See 'vehicle_dynamics.png/tex' for documentation.
    % Cross check with vehicle paper
    
    px    = x(:,1);
    py    = x(:,2);
    yaw   = x(:,3);
    v     = x(:,4);
    
    f          = u(:,1);
    delta_ref  = u(:,2);
    
    delta = delta_ref + p(8);
    
    dx = 0*x;
    
    dx(:,1) = p(1) .* v .* (1 + p(2) .* delta.^2) .* cos(yaw + p(3) .* delta + p(9));
    dx(:,2) = p(1) .* v .* (1 + p(2) .* delta.^2) .* sin(yaw + p(3) .* delta + p(9));
    dx(:,3) = p(4) .* v .* delta;
    dx(:,4) = p(5) .* v + p(6) .* sign(f) .* (abs(f).^(p(7)));
    
end

