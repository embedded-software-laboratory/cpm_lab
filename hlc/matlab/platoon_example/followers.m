function [msg]=followers(vehicle_id, state_list, follow_id, t_now)
    %% Do not display figures
    set(0,'DefaultFigureVisible','off');

    % Get time (same as in trajectory_complex)
    % t_now = getTimestampNow();
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