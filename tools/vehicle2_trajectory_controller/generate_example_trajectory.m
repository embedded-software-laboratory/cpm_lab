function generate_example_trajectory
   
    log = importdata('vehicleStateLog.csv');   
    
    Timestamp = uint64(log.data(:, strcmp(log.colheaders, 'Timestamp'))); 
    
    px = log.data(:, strcmp(log.colheaders, 'pose.x'));
    py = log.data(:, strcmp(log.colheaders, 'pose.y'));
    yaw = log.data(:, strcmp(log.colheaders, 'pose.yaw'));
    v = log.data(:, strcmp(log.colheaders, 'speed'));
    v = medfilt1(v, 5);
    
    vx = v .* cos(yaw);
    vy = v .* sin(yaw);
    
    start_idx = find(v > 0.1);
    start_idx = start_idx(1);
    slice = start_idx:20:(start_idx+550);
    
    T_traj = uint64(Timestamp(slice));
    Timestamp = Timestamp - uint64(min(T_traj));
    T_traj = uint64(T_traj - min(T_traj));
    px_traj = px(slice);
    py_traj = py(slice);
    vx_traj = vx(slice);
    vy_traj = vy(slice);
    
    cycle_start = 10;
    tmp_ds = norm([px_traj(cycle_start) py_traj(cycle_start)]-[px_traj(end) py_traj(end)]);
    tmp_v = norm([vx_traj(cycle_start) vy_traj(cycle_start)]);
    cycle_closing_segment_time = 1e9 * tmp_ds / tmp_v;
    n_trajectory = length(slice);
    
    
    % expand trajectory cycle
    for i = 1:20
        cycle_time = uint64(T_traj(end) - T_traj(cycle_start) + cycle_closing_segment_time);
        T_traj = uint64([T_traj; (T_traj(cycle_start:n_trajectory) + cycle_time)]);
        px_traj = [px_traj; px_traj(cycle_start:n_trajectory)];
        py_traj = [py_traj; py_traj(cycle_start:n_trajectory)];
        vx_traj = [vx_traj; vx_traj(cycle_start:n_trajectory)];
        vy_traj = [vy_traj; vy_traj(cycle_start:n_trajectory)];        
    end
    
    
    % generate example input data
    fID = fopen('example_trajectory.hpp', 'w');    
    
    fprintf(fID,'const uint64_t example_trajectory_timestamp_offset[] = { ');
    fprintf(fID,'%u, ', uint64(T_traj));
    fprintf(fID,'};\n');
    
    fprintf(fID,'const double example_trajectory_px[] = { ');
    fprintf(fID,'%f, ', px_traj);
    fprintf(fID,'};\n');
    
    fprintf(fID,'const double example_trajectory_py[] = { ');
    fprintf(fID,'%f, ', py_traj);
    fprintf(fID,'};\n');
    
    fprintf(fID,'const double example_trajectory_vx[] = { ');
    fprintf(fID,'%f, ', vx_traj);
    fprintf(fID,'};\n');
    
    fprintf(fID,'const double example_trajectory_vy[] = { ');
    fprintf(fID,'%f, ', vy_traj);
    fprintf(fID,'};\n');
    
    fprintf(fID,'const int example_trajectory_size = %i;\n', length(vy_traj));
    
    fclose(fID);
    
    
    
    
    
    
    % calculate interpolation
    t_interp = uint64((0:0.005:30)*1e9);
    position_x_interp     = nan(size(t_interp));
    position_y_interp     = nan(size(t_interp));
    velocity_x_interp     = nan(size(t_interp));
    velocity_y_interp     = nan(size(t_interp));
    acceleration_x_interp = nan(size(t_interp));
    acceleration_y_interp = nan(size(t_interp));
    yaw_interp            = nan(size(t_interp));
    speed_interp          = nan(size(t_interp));
    curvature_interp      = nan(size(t_interp));
    
    
    for i = 1:length(t_interp)
        t_now = t_interp(i);
        I = find(t_now < T_traj);
        I = I(1)-1;
        
        assert (T_traj(I) <= t_now)
        assert (T_traj(I+1) >= t_now)

        [...
            position_x_interp(i), ...
            position_y_interp(i), ...
            velocity_x_interp(i), ...
            velocity_y_interp(i), ...
            acceleration_x_interp(i), ...
            acceleration_y_interp(i), ...
            yaw_interp(i), ...
            speed_interp(i), ...
            curvature_interp(i)...
        ] ...
        = trajectory_interpolation(...
            double(t_now)/1e9,...
            double(T_traj(I))/1e9, ...
            px_traj(I), ...
            py_traj(I), ...
            vx_traj(I), ...
            vy_traj(I), ...
            double(T_traj(I+1))/1e9, ...
            px_traj(I+1), ...
            py_traj(I+1), ...
            vx_traj(I+1), ...
            vy_traj(I+1) ...
        );
    end
    
    
    clf
    
    subplot(2,1,1)
    hold on
    grid on
    axis equal    
    scatter(px_traj, py_traj)
    plot(position_x_interp, position_y_interp)
    
    
    subplot(2,1,2)
    hold on
    grid on
    scatter(t_interp, speed_interp)
    
    
end

