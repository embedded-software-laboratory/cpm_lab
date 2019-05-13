function manual_parameter_fit

    % Run 'recording_to_csv.bash' before this script!
    
    % This is a first experiment and sanity check for the parameter
    % identification. The model is simulated with real command 
    % input values and manually estimated parameters.
    % This should show that the model produces somewhat sensible
    % trajectories and not complete nonsense.
    % These parameters server as the initialization for the optimization.
    
    ips = read_rti_csv('output/recording_vehicle_3_bdat_0_0_vehicleObservation_RecordAll_domain0.csv');
    state = read_rti_csv('output/recording_vehicle_3_bdat_0_0_vehicleState_RecordAll_domain0.csv');
    
    t_ips = ips.header_create_stamp_nanoseconds * 1e-9;
    t_state = state.header_create_stamp_nanoseconds * 1e-9;

    t_0 = max(t_state(1), t_ips(1)) + 2;
    t_f = min(t_state(end), t_ips(end)) - 2;
    
    t_f = min(t_f, t_0 + 10); % crop time period
    
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
    X = [...
        ips.pose_x(idx_ips(1))  ...
        ips.pose_y(idx_ips(1))  ...
        ips.pose_yaw(idx_ips(1))  ...
        state.speed(idx_state(1))  ...
        .3 ...
    ];
    params = zeros(1,10);
    T_v = 0.4;
    params(1) = 0.94;
    params(4) = 1/0.34;
    params(5) = -1/T_v;
    params(6) = 6.6/T_v;
    params(8) = 1.62;
    params(9) = 1/0.18;
    
    for i = 2:length(t_grid)
        
        u = [state.motor_throttle(idx_state(i-1)) ...
             state.steering_servo(idx_state(i-1)) ...    
             state.battery_voltage(idx_state(i-1))];
        
        dx = vehicle_dynamics(X(i-1,:),u,params);
        
        X(i,:) = X(i-1,:) + dt * dx;
        
    end
    
    
    % Visualize
    clf
    hold on
    box on
    grid on
    axis equal
    
    
    plot(ips.pose_x(idx_ips), ips.pose_y(idx_ips))
    plot(X(:,1), X(:,2))
    
    
end

