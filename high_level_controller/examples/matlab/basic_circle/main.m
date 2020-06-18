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
    matlabDomainID = 1;
    
    % Import IDL files from cpm library
    dds_idl_matlab = fullfile('../../../../cpm_lib/dds_idl_matlab/');
    assert(isfolder(dds_idl_matlab),...
        'Missing directory "%s".', dds_idl_matlab);
    assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
        'No MATLAB IDL-files found in %s', dds_idl_matlab);
    addpath(dds_idl_matlab)

    % XML files for quality of service settings
    middleware_local_qos_xml = '../../../../middleware/build/QOS_LOCAL_COMMUNICATION.xml';
    assert(isfile(middleware_local_qos_xml),...
        'Missing middleware local QOS XML "%s"', middleware_local_qos_xml);
    
    ready_trigger_qos_xml = '../QOS_READY_TRIGGER.xml';
    assert(isfile(ready_trigger_qos_xml),...
        'Missing ready trigger QOS XML "%s"', ready_trigger_qos_xml);
    
    setenv("NDDS_QOS_PROFILES", ['file://' ready_trigger_qos_xml ';file://' middleware_local_qos_xml]);
    
    %% variables for DDS communication
    % DDS Participant for local communication with middleware
    matlabParticipant = DDS.DomainParticipant(...
        'MatlabLibrary::LocalCommunicationProfile',...
        matlabDomainID);
    % Infrastructure
    topic_readyStatus = 'readyStatus';
    writer_readyStatus = DDS.DataWriter(...
        DDS.Publisher(matlabParticipant),...
        'ReadyStatus',...
        topic_readyStatus,...
        'TriggerLibrary::ReadyTrigger');

    topic_systemTrigger = 'systemTrigger';
    reader_systemTrigger = DDS.DataReader(...
        DDS.Subscriber(matlabParticipant),...
        'SystemTrigger',...
        topic_systemTrigger,...
        'TriggerLibrary::ReadyTrigger');
    trigger_stop = uint64(18446744073709551615);

    % Control
    topic_vehicleStateList = 'vehicleStateList';
    reader_vehicleStateList = DDS.DataReader(...
        DDS.Subscriber(matlabParticipant),...
        'VehicleStateList',...
        topic_vehicleStateList);

    topic_vehicleCommandTrajectory = 'vehicleCommandTrajectory';
    writer_vehicleCommandTrajectory = DDS.DataWriter(...
        DDS.Publisher(matlabParticipant),...
        'VehicleCommandTrajectory',...
        topic_vehicleCommandTrajectory);
    
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
    reader_vehicleStateList.WaitSetTimeout = 60;

    % Define reference trajectory
    reference_trajectory_index = 1;
    reference_trajectory_time = 0;
    map_center_x = 2.25;
    map_center_y = 2.0;
    trajectory_px    = [         1,          0,         -1,          0] + map_center_x;
    trajectory_py    = [         0,          1,          0,         -1] + map_center_y;
    trajectory_vx    = [         0,         -1,          0,          1];
    trajectory_vy    = [         1,          0,         -1,          0];
    segment_duration = [1550000000, 1550000000, 1550000000, 1550000000];
    
    while (~got_stop)
        % Read vehicle states
        [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
        assert(sample_count == 1, 'Received %d samples, expected 1', sample_count);
        fprintf('Received sample at time: %d\n',sample.t_now);
        
        if (reference_trajectory_time == 0)
            reference_trajectory_time = sample.t_now;
        end
        
        % Create reference trajectory
        t_ahead_nanos = 0;
        i_traj_index = reference_trajectory_index;

        trajectory_points = [];
        plan_ahead_time_nanos = 7000000000;
        while (t_ahead_nanos < plan_ahead_time_nanos)
            the_trajectory_point = TrajectoryPoint;
            the_trajectory_point.t.nanoseconds = uint64(reference_trajectory_time + t_ahead_nanos);
            the_trajectory_point.px = trajectory_px(i_traj_index);
            the_trajectory_point.py = trajectory_py(i_traj_index);
            the_trajectory_point.vx = trajectory_vx(i_traj_index);
            the_trajectory_point.vy = trajectory_vy(i_traj_index);
            trajectory_points = [trajectory_points the_trajectory_point];
            t_ahead_nanos = t_ahead_nanos + segment_duration(i_traj_index);
            i_traj_index = mod(i_traj_index, length(segment_duration)) + 1;
        end
            
        % Send the current trajectory point to the vehicle
        max_delay_time_nanos = 200000000;
        vehicle_command_trajectory = VehicleCommandTrajectory;
        vehicle_command_trajectory.vehicle_id = uint8(vehicle_id);
        vehicle_command_trajectory.trajectory_points = trajectory_points;
        vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
            uint64(sample.t_now);
        vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
            uint64(sample.t_now + max_delay_time_nanos);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

        % The vehicle always needs a trajectory point at or before the current time,
        % as well as enough trajectory points in the future,
        % to allow some time for the vehicle to receive
        % the message and anticipate the next turn.
        next_reference_trajectory_time = ...
            reference_trajectory_time + segment_duration(reference_trajectory_index);
        while (next_reference_trajectory_time < sample.t_now)
            reference_trajectory_time = ...
                reference_trajectory_time + segment_duration(reference_trajectory_index);
            reference_trajectory_index = ...
                mod(reference_trajectory_index, length(segment_duration)) + 1;
            next_reference_trajectory_time = ...
                reference_trajectory_time + segment_duration(reference_trajectory_index);
        end
        
        % Check for stop signal
        [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    end
end

function [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop)
    [trigger, ~, sample_count, ~] = reader_systemTrigger.take();
    got_stop = false;
    got_start = false;
    if sample_count > 0
        % look at most recent signal with (end)
        if trigger(end).next_start().nanoseconds() == trigger_stop
            got_stop = true;
        else
            got_start = true;
        end
    end
end