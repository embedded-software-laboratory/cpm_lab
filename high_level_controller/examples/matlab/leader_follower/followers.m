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

function [msg]=followers(vehicle_id, state_list, follow_id, t_now)
    %% Do not display figures
    set(0,'DefaultFigureVisible','off');

    % Get time (same as in trajectory_complex)
    point_period_nanoseconds = 250000000;
    t_eval = ((t_now + uint64(500000000)) / point_period_nanoseconds) * point_period_nanoseconds;

    % Create msg
    trajectory = VehicleCommandTrajectory;
    trajectory.vehicle_id = uint8(vehicle_id);
    trajectory_points = [];
    point1 = TrajectoryPoint;

    % Search for vehicle position (follow_id) in state list
    follow_state = VehicleState;
    has_follow_state = false;
    for i = 1:length(state_list)
        if state_list(i).vehicle_id == follow_id
            has_follow_state = true;
            follow_state = state_list(i);
            break;
        end
    end

    % Rest of msg
    time = t_eval + 400000000;
    stamp = TimeStamp;
    stamp.nanoseconds = uint64(time);
    point1.t = stamp;

    % Only send a valid message if the state value of the vehicle to follow exists
    if has_follow_state
        point1.px = follow_state.pose.x;
        point1.py = follow_state.pose.y;
        point1.vx = cos(follow_state.pose.yaw) * follow_state.speed;
        point1.vy = sin(follow_state.pose.yaw) * follow_state.speed;
    else
        stamp2 = TimeStamp;
        stamp2.nanoseconds = uint64(0);
        point1.t = stamp2;
    end

    trajectory_points = [trajectory_points [point1]];
    trajectory.trajectory_points = trajectory_points;

    % Return msg
    msg = trajectory;
end