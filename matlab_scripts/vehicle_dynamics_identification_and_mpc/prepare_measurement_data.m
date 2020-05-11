function prepare_measurement_data

    %% Run 'recording_to_mat.bash' before this script!
    
    % This script takes the recorded data and filters, combines and slices it in
    % preparation for the optimization.
    
    % Number of consecutive samples in a sequence (at 50Hz)
    n_sequence_samples = 100;
    
    % The optimization is done individually for each vehicle
    vehicle_id = 2;
    
    recording_file_names = {'recording_vehicles_2_3_test_loop', 'recording_vehicles_2_3_test_loop_b'};
    
    freq = 50; % assume data comes at 50Hz
    sequence_duration = n_sequence_samples / freq;
    
    result_sequences = struct;
    
    for idx_recording = 1:length(recording_file_names)
               
        DataByVeh = load(recording_file_names{idx_recording});
        ips = DataByVeh.databyveh.(['vehicle_' int2str(vehicle_id)]).observation;
        state = DataByVeh.databyveh.(['vehicle_' int2str(vehicle_id)]).state;

        % Timestamps
        t_state = state.create_stamp;
        t_ips = ips.create_stamp;
        
        assert(all(diff(t_state) > 0))
        assert(all(diff(t_ips) > 0))
        
        % Find intervals of continuous data
        idx_jump_ips = find(diff(t_ips) > 1.1/freq);        
        idx_interval_start_ips = [1; idx_jump_ips]+2;
        idx_interval_end_ips = [idx_jump_ips; length(t_ips)]-2;
        filter_ips = ((idx_interval_end_ips - idx_interval_start_ips) > n_sequence_samples);
        idx_interval_start_ips = idx_interval_start_ips(filter_ips);
        idx_interval_end_ips = idx_interval_end_ips(filter_ips);
        t_interval_start_ips = t_ips(idx_interval_start_ips);
        t_interval_end_ips = t_ips(idx_interval_end_ips);
        
        
        idx_jump_state = find(diff(t_state) > 1.1/freq);
        idx_interval_start_state = [1; idx_jump_state]+2;
        idx_interval_end_state = [idx_jump_state; length(t_state)]-2;
        filter_state = ((idx_interval_end_state - idx_interval_start_state) > n_sequence_samples);
        idx_interval_start_state = idx_interval_start_state(filter_state);
        idx_interval_end_state = idx_interval_end_state(filter_state);
        t_interval_start_state = t_state(idx_interval_start_state);
        t_interval_end_state = t_state(idx_interval_end_state);
        
        % Find interval intersections between vehicle and IPS data
        valid_intervals = and((t_interval_end_state - t_interval_start_ips' > sequence_duration), ...
                              (t_interval_end_ips' - t_interval_start_state > sequence_duration));
                          
        [idx_t_interval_state, idx_t_interval_ips] = find(valid_intervals);
        
        t_interval_start = max(t_interval_start_state(idx_t_interval_state), t_interval_start_ips(idx_t_interval_ips));
        t_interval_end = min(t_interval_end_state(idx_t_interval_state), t_interval_end_ips(idx_t_interval_ips));
        
        assert(all(t_interval_end - t_interval_start > sequence_duration));
        
        % Split intervals into shorter ones of length 'sequence_duration'
        t_interval_short_start = [];
        t_interval_short_end = [];
        n_interval_subsegments = floor((t_interval_end - t_interval_start) / sequence_duration);
        for j = 1:length(n_interval_subsegments)
             t_interval_short_start = [t_interval_short_start ...
                 (t_interval_start(j) + sequence_duration * ((1:n_interval_subsegments(j)) - 1))];
             
             t_interval_short_end = [t_interval_short_end ...
                 (t_interval_start(j) + sequence_duration * ((1:n_interval_subsegments(j))))];
        end
        
        % Collect data for each interval
        for j = 1:length(t_interval_short_start)
            t_offset = 1e-5;
            filter_ips = and(t_interval_short_start(j) - t_offset <= t_ips, t_ips < t_interval_short_end(j) - t_offset);
            filter_state = and(t_interval_short_start(j) - t_offset <= t_state, t_state < t_interval_short_end(j) - t_offset);
            
            assert(nnz(filter_ips) == n_sequence_samples)
            assert(nnz(filter_state) == n_sequence_samples)
            
            n = length(result_sequences) + 1;
            
            result_sequences(n).x = ips.x(filter_ips);
            result_sequences(n).y = ips.y(filter_ips);
            result_sequences(n).yaw = ips.yaw(filter_ips);
            result_sequences(n).speed = state.speed(filter_state);
            result_sequences(n).steering_command = state.steering_servo(filter_state);
            result_sequences(n).motor_command = state.motor_throttle(filter_state);
            result_sequences(n).battery_voltage = state.battery_voltage(filter_state);
        end
        



    end
    
%     i = ceil(length(result_sequences) * rand);
%     clf
%     plot(result_sequences(i).x, result_sequences(i).y)
%     axis equal
%     grid on
    
    mkdir('output');
    sequences = result_sequences(2:end);
    save('output/sequences', 'sequences');
    
end

