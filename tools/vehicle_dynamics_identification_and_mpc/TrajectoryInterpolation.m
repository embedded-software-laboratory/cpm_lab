% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function interp = TrajectoryInterpolation(t_now, start_point, end_point)
    
    t_start = (start_point.t) / 1e9;
    t_end = (end_point.t) / 1e9;

    delta_t = t_end - t_start;
    tau = (t_now - t_start) / delta_t;
    
    tau2 = tau * tau;
    tau3 = tau * tau2;
    
    position_start_x = start_point.px;
    position_start_y = start_point.py;
    position_end_x = end_point.px;
    position_end_y = end_point.py;
    
    velocity_start_x = start_point.vx * delta_t;
    velocity_start_y = start_point.vy * delta_t;
    velocity_end_x = end_point.vx * delta_t;
    velocity_end_y = end_point.vy * delta_t;
    
    
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
    
    position_x     =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    position_y     =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
    velocity_x     = (position_start_x *  dp0 + velocity_start_x *  dm0 + position_end_x *  dp1 + velocity_end_x *  dm1) / delta_t;
    velocity_y     = (position_start_y *  dp0 + velocity_start_y *  dm0 + position_end_y *  dp1 + velocity_end_y *  dm1) / delta_t;
    acceleration_x = (position_start_x * ddp0 + velocity_start_x * ddm0 + position_end_x * ddp1 + velocity_end_x * ddm1) / (delta_t*delta_t);
    acceleration_y = (position_start_y * ddp0 + velocity_start_y * ddm0 + position_end_y * ddp1 + velocity_end_y * ddm1) / (delta_t*delta_t);
    
    yaw = atan2(velocity_y, velocity_x);
    speed = sqrt(velocity_x*velocity_x + velocity_y*velocity_y);
    curvature = (velocity_x * acceleration_y - velocity_y * acceleration_x) / (speed*speed*speed);
    
    interp.position_x = position_x;
    interp.position_y = position_y;
    interp.velocity_x = velocity_x;
    interp.velocity_y = velocity_y;
    interp.acceleration_x = acceleration_x;
    interp.acceleration_y = acceleration_y;
    interp.yaw = yaw;
    interp.speed = speed;
    interp.curvature = curvature;
    
end

