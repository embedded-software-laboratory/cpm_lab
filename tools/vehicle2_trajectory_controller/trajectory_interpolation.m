function [...
    position, ...
    velocity, ...
    acceleration, ...
    yaw, ...
    speed, ...
    curvature...
    ] ...
    = trajectory_interpolation(...
t_interp,...
t_start, ...
position_start, ...
velocity_start, ...
t_end, ...
position_end, ...
velocity_end...
)

    delta_t = t_end - t_start;
    tau = (t_interp - t_start) ./ delta_t;
    
    tau2 = tau .* tau;
    tau3 = tau .* tau2;
    
    velocity_start = velocity_start * delta_t;
    velocity_end = velocity_end * delta_t;
    
    
    % Hermite spline coefficients
    p0 = 2*tau3 - 3*tau2 + 1;
    m0 = tau3 - 2*tau2 + tau;
    p1 = -2*tau3 + 3*tau2;
    m1 = tau3 - tau2;
    
    % Hermite spline derivative coefficients
    dp0 = 6*tau2 - 6*tau;
    dm0 = 3*tau2 - 4*tau + 1;
    dp1 = -6*tau2 + 6*tau;
    dm1 = 3*tau2 - 2*tau;
    
    % Hermite spline second derivative coefficients
    ddp0 = 12*tau - 6;
    ddm0 = 6*tau - 4;
    ddp1 = -12*tau + 6;
    ddm1 = 6*tau - 2;    
    
    position     =  position_start *   p0 + velocity_start *   m0 + position_end *   p1 + velocity_end *   m1;
    velocity     = (position_start *  dp0 + velocity_start *  dm0 + position_end *  dp1 + velocity_end *  dm1) / delta_t;
    acceleration = (position_start * ddp0 + velocity_start * ddm0 + position_end * ddp1 + velocity_end * ddm1) / (delta_t*delta_t);
    
    yaw = atan2(velocity(2,:), velocity(1,:));
    speed = sqrt(velocity(1,:).^2 + velocity(2,:).^2);
    
    curvature = (velocity(1,:) .* acceleration(2,:) - velocity(2,:) .* acceleration(1,:)) ./ (speed.^3);
    
    
end

