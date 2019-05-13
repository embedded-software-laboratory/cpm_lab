function manual_parameter_fit
    
    ips = read_rti_csv('output/recordingvehicle3bdat00_vehicleObservation_RecordAll_domain0.csv');
    state = read_rti_csv('output/recordingvehicle3bdat00_vehicleState_RecordAll_domain0.csv');
    
    t_ips = ips.header_create_stamp_nanoseconds * 1e-9;
    t_state = state.header_create_stamp_nanoseconds * 1e-9;

    t_0 = max(t_state(1), t_ips(1)) + 2;
    t_f = min(t_state(end), t_ips(end)) - 2;
    
    t_f = min(t_f, t_0 + 10); % show only 10 sec
    
    freq = 50;
    
    t_0 = ceil(t_0*freq)/freq;
    dt = 1/freq;
    
    % Simulation grid
    t_grid = t_0:dt:t_f;
    
    % Index mapping from simulation grid to data grids
    idx_ips = nan(size(t_grid));
    idx_state = nan(size(t_grid));
    
    for i = 1:length(t_grid)
        [v,I] = min(abs(t_grid(i) - t_ips));
        assert(v < 1.1 / freq);
        idx_ips(i) = I(1);
        
        
        [v,I] = min(abs(t_grid(i) - t_state));
        assert(v < 1.1 / freq);
        idx_state(i) = I(1);
    end
    
    assert(~any(isnan(idx_ips)));
    assert(~any(isnan(idx_state)));
    
    % Simulate
    X0 = [...
        ips.pose_x(idx_ips(1))  ...
        ips.pose_y(idx_ips(1))  ...
        ips.pose_yaw(idx_ips(1))  ...
        state.speed(idx_state(1))  ...
        0 ...
    ];
    params = zeros(1,10);
    params(1) = 1;
    params(4) = 1/0.24;
    
    
    
    % Visualize
    clf
    hold on
    box on
    grid on
    axis equal
    
    
    plot(ips.pose_x(idx_ips), ips.pose_y(idx_ips))
    
    
    
end

