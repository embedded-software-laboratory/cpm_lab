function prepare_measurement_data

    % Run 'recording_to_csv.bash' before this script!
    
    % This script takes the recorded data and filters, combines and slices it in
    % preparation for the optimization.
    
    % Number of consecutive samples in a sequence (at 50Hz)
    n_sequence_samples = 300;
    
    % The optimization is done individually for each vehicle
    vehicle_id = 3;
    
    recording_file_prefixes = {'recording_vehicle_3_a', 'recording_vehicle_3_b', 'recording_vehicle_3_c'};
    
    freq = 50; % assume data comes at 50Hz
    sequence_duration = n_sequence_samples / freq;
    
    for idx_recording = 1:length(recording_file_prefixes)
        
        file_state = ['output/' recording_file_prefixes{idx_recording} 'dat_0_0_vehicleState_RecordAll_domain0.csv'];
        file_ips = ['output/' recording_file_prefixes{idx_recording} 'dat_0_0_vehicleObservation_RecordAll_domain0.csv'];
        
        % Load data
        state = read_rti_csv(file_state);
        ips = read_rti_csv(file_ips);
        
        % Filter vehicle ID
        ips = filter_struct(ips, ips.vehicle_id == 3);
        state = filter_struct(state, state.vehicle_id == 3);

        % Timestamps
        t_state = state.header_create_stamp_nanoseconds * 1e-9;
        t_ips = ips.header_create_stamp_nanoseconds * 1e-9;
        
        % Find intervals of continuous data
        idx_jump_ips = find(diff(t_ips) > 1.1/freq);        
        idx_interval_start_ips = [1; idx_jump_ips]+2;
        idx_interval_end_ips = [idx_jump_ips; length(ips.header_create_stamp_nanoseconds)]-2;
        filter_ips = ((idx_interval_end_ips - idx_interval_start_ips) > n_sequence_samples);
        idx_interval_start_ips = idx_interval_start_ips(filter_ips);
        idx_interval_end_ips = idx_interval_end_ips(filter_ips);
        t_interval_start_ips = t_ips(idx_interval_start_ips);
        t_interval_end_ips = t_ips(idx_interval_end_ips);
        
        
        idx_jump_state = find(diff(t_state) > 1.1/freq);
        idx_interval_start_state = [1; idx_jump_state]+2;
        idx_interval_end_state = [idx_jump_state; length(state.header_create_stamp_nanoseconds)]-2;
        filter_state = ((idx_interval_end_state - idx_interval_start_state) > n_sequence_samples);
        idx_interval_start_state = idx_interval_start_state(filter_state);
        idx_interval_end_state = idx_interval_end_state(filter_state);
        t_interval_start_state = t_state(idx_interval_start_state);
        t_interval_end_state = t_state(idx_interval_end_state);
        
        % Find interval intersections between vehicle and IPS data
        

        % TODO
        % filter ID
        % match timelines
        % skip parts with missing data
        % partition recording
        % LP filter voltage?


    end
    
end

