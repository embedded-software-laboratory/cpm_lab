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

function main(vehicle_id)
    % Initialize data readers/writers...
    common_cpm_functions_path = fullfile( ...
        getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    );
    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);
    
    matlabDomainId = 1;
    % matlabParticipant implicitly needed
    [matlabParticipant, reader_vehicleStateList, ~, writer_vehicleCommandPathTracking, reader_systemTrigger, writer_readyStatus, trigger_stop] = init_script(matlabDomainId);


    %% Sync start with infrastructure
    % Send ready signal
    % Signal needs to be sent for all assigned vehicle ids
    % Also for simulated time case - period etc are set in Middleware,
    % so timestamp field is meaningless
    disp('Sending ready signal');
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(vehicle_id));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    writer_readyStatus.write(ready_msg);

    % Wait for start or stop signal
    disp('Waiting for start or stop signal');    
    got_stop = false;
    got_start = false;
    while (~got_stop && ~got_start)
        [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    end
    

    %% Run the HLC
    % Set reader properties
    reader_vehicleStateList.WaitSet = true;
    reader_vehicleStateList.WaitSetTimeout = 5; % [s]

    % Reference path generation
    path_points = get_path_points('circle');
    
    % Middleware period for valid_after stamp
    dt_period_nanos = 250e6;

    % Main control loop
    while (~got_stop)
        % Read vehicle states
        [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
        if (sample_count > 1)
            warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            sample = sample(end); % Use latest sample
        end
        fprintf('Received sample at time: %d\n',sample.t_now);
        
        vehicle_command_path_tracking = VehicleCommandPathTracking;
        vehicle_command_path_tracking.vehicle_id = uint8(vehicle_id);
        vehicle_command_path_tracking.path = path_points;
        vehicle_command_path_tracking.speed = 1.0;
        vehicle_command_path_tracking.header.create_stamp.nanoseconds = ...
            uint64(sample(end).t_now);
        vehicle_command_path_tracking.header.valid_after_stamp.nanoseconds = ...
            uint64(sample(end).t_now + dt_period_nanos);
        writer_vehicleCommandPathTracking.write(vehicle_command_path_tracking);
        
        % Check for stop signal
        [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    end
end
