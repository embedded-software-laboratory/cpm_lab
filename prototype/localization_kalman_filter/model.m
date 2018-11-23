function [dx, z_L, z_P] = model(x, u)

    px          = x(1);
    py          = x(2);
    yaw         = x(3);
    yaw_offset  = x(4); % For the IMU drift
    s           = x(5);
    v           = x(6);
    curvature   = x(7);
    
    v_ref          = u(1);
    curvature_ref  = u(2);
    
    T_curvature = 0.09;
    T_speed = 0.7;

    d_px           = v * cos(yaw);
    d_py           = v * sin(yaw);
    d_yaw          = v * curvature;
    d_yaw_offset   = 0;
    d_s            = v;
    d_v            = (v_ref - v)/T_speed;
    d_curvature    = (curvature_ref - curvature)/T_curvature;
    
    dx = [d_px; d_py; d_yaw; d_yaw_offset; d_s; d_v; d_curvature];
    
    % local measurement
    z_L = [ (yaw+yaw_offset); s ];
    
    % local+IPS measurement
    z_P = [ (yaw+yaw_offset); s; px; py; yaw ];
end

