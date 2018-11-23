function [dx, z] = model(x, u)

    px          = x(1);
    py          = x(2);
    yaw_offset  = x(3); % For the IMU drift
    
    v          = u(1);
    yaw_imu    = u(2);
    

    d_px           = v * cos(yaw_imu + yaw_offset);
    d_py           = v * sin(yaw_imu + yaw_offset);
    d_yaw_offset   = 0;
    
    dx = [d_px; d_py; d_yaw_offset];
    
    % predicted IPS measurement
    z = [ px; py; yaw_imu + yaw_offset ];
end

