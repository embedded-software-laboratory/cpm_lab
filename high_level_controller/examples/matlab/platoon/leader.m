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