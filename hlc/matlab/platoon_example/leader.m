function [msg]=leader(vehicle_id, t_now)
    %% Do not display figures
    set(0,'DefaultFigureVisible','off');

    % Get data index
    % t_now = getTimestampNow();
    point_period_nanoseconds = 250000000;
    t_eval = ((t_now + uint64(500000000)) / point_period_nanoseconds) * point_period_nanoseconds;

    trajectory_index = (t_eval) / point_period_nanoseconds;

    trajectory_point = trajectory_test_loop(trajectory_index);

    %Create msg
    trajectory = VehicleCommandTrajectory;
    trajectory.vehicle_id = uint8(vehicle_id);
    trajectory_points = [];
    point1 = TrajectoryPoint;

    time = t_eval + 400000000;
    stamp = TimeStamp;
    stamp.nanoseconds = uint64(time);
    point1.t = stamp;
    point1.px = trajectory_point(1);
    point1.py = trajectory_point(2);
    point1.vx = trajectory_point(3);
    point1.vy = trajectory_point(4);

    trajectory_points = [trajectory_points [point1]];
    trajectory.trajectory_points = trajectory_points;

    % Return msg
    msg = trajectory;
end