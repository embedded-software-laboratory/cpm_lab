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

function [trjMsg, stufMsg] = trajMessage(trajectory_points, vehicle_id, t_now)
    %% Do not display figures
    set(0,'DefaultFigureVisible','off');
    
    trajectory = VehicleCommandTrajectory;
    trajectory.vehicle_id = uint8(vehicle_id);
    trajectory.header.create_stamp.nanoseconds = t_now;
    trajectory.header.valid_after_stamp.nanoseconds = t_now - 40000000;
    driven_segment_time = zeros(1,length(trajectory_points));
 
    for nPoints = 1:length(trajectory_points)
        %nochmam überdenken und anpassen
        driven_segment_time(nPoints) = trajectory_points(nPoints).t;
        time = t_now + sum(driven_segment_time);
        stamp = TimeStamp;
        stamp.nanoseconds = uint64(time);
        trajectory_points(nPoints).t = stamp;

    end
    
    trajectory.trajectory_points = trajectory_points;
    
    stuffingTraj = VehicleCommandTrajectory;
    stuffingTraj.vehicle_id = uint8(vehicle_id);
    stuffingTraj.header.create_stamp.nanoseconds = t_now;
    stuffingTraj.header.valid_after_stamp.nanoseconds = t_now + 1000000000000;
    stuffingTraj.trajectory_points = trajectory_points(end);
    
    % Return msg
    trjMsg = trajectory;
    stufMsg = stuffingTraj;
end