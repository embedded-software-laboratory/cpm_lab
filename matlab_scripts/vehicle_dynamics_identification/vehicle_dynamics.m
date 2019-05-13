function dx = vehicle_dynamics(x,u,p)
    
    px    = x(:,1);
    py    = x(:,2);
    yaw   = x(:,3);
    v     = x(:,4);
    delta = x(:,5);
    
    f          = u(:,1);
    delta_ref  = u(:,2);
    V          = u(:,3);
    
    dx = 0*x;
    
    dx(:,1) = p(1) .* v .* (1 + p(2) .* delta.^2) .* cos(yaw + p(3) .* delta + p(11));
    dx(:,2) = p(1) .* v .* (1 + p(2) .* delta.^2) .* sin(yaw + p(3) .* delta + p(11));
    dx(:,3) = p(4) .* v .* delta;
    dx(:,4) = p(5) .* v + (p(6) + p(7) .* V) .* sign(f) .* (abs(f).^(p(8)));
    dx(:,5) = p(9) .* (-delta + delta_ref + p(10));
    
end

