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

function [msg]=leader(vehicle_slot, t_now)
    %% Do not display figures
    set(0,'DefaultFigureVisible','off');

    % Get data index
    point_period_nanoseconds = 250000000;
    t_eval = ((t_now + uint64(500000000)) / point_period_nanoseconds) * point_period_nanoseconds;

    trajectory_index = (t_eval) / point_period_nanoseconds;
    % Add vehicle ID to data index so that different cars have slightly different trajectories
    trajectory_index = trajectory_index + vehicle_slot * 2;

    %Create msg
    trajectory = VehicleCommandTrajectory;
    trajectory.header.create_stamp.nanoseconds = t_eval;
    trajectory.header.valid_after_stamp.nanoseconds = t_eval + 400000000;
    trajectory_points = [];

    for i = 0 : 3
        % Get current trajectory from pre-computed trajectory list
        trajectory_point = leader_trajectory(trajectory_index + (i - 1));

        point1 = TrajectoryPoint;

        time = t_eval + i * 400000000;
        stamp = TimeStamp;
        stamp.nanoseconds = uint64(time);
        point1.t = stamp;
        point1.px = trajectory_point(1);
        point1.py = trajectory_point(2);
        point1.vx = trajectory_point(3);
        point1.vy = trajectory_point(4);

        trajectory_points = [trajectory_points [point1]];
    end
    trajectory.trajectory_points = trajectory_points;

    % Return msg
    msg = trajectory;
end