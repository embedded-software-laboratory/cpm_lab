function main(matlabDomainID, vehicle_id)    
    % Get all relevant files - IDL files for communication, XML files for communication settings etc
    middleware_local_qos_xml = '../../middleware/build/QOS_LOCAL_COMMUNICATION.xml';
    if ~exist(middleware_local_qos_xml,'file')
        error(['Missing middleware local QOS XML "' middleware_local_qos_xml '"'])
    end    
    setenv("NDDS_QOS_PROFILES", ['file://' pwd '/../QOS_READY_TRIGGER.xml;file://' middleware_local_qos_xml]);
            
    %% Import IDL files
    run('../import_dds_idl.m');
    
    %% variables for the communication
    matlabStateTopicName = 'vehicleStateList';
    matlabCommandTopicName = 'vehicleCommandTrajectory';
    systemTriggerTopicName = 'systemTrigger';
    readyStatusTopicName = 'readyStatus';
    trigger_stop = uint64(18446744073709551615);

    %% create DDS participant
    matlabParticipant = DDS.DomainParticipant(...
        'MatlabLibrary::LocalCommunicationProfile',...
        matlabDomainID);
    % create DDS readers and writers
    systemTriggerReader = DDS.DataReader(...
        DDS.Subscriber(matlabParticipant),...
        'SystemTrigger',...
        systemTriggerTopicName,...
        'TriggerLibrary::ReadyTrigger');
    stateReader = DDS.DataReader(...
        DDS.Subscriber(matlabParticipant),...
        'VehicleStateList',...
        matlabStateTopicName);
    readyStatusWriter = DDS.DataWriter(...
        DDS.Publisher(matlabParticipant),...
        'ReadyStatus',...
        readyStatusTopicName,...
        'TriggerLibrary::ReadyTrigger');
    writer_vehicleCommandTrajectory = DDS.DataWriter(...
        DDS.Publisher(matlabParticipant),...
        'VehicleCommandTrajectory',...
        matlabCommandTopicName);
    
    %% Send ready signal
    % Signal needs to be sent for all assigned vehicle ids
    % Also for simulated time case - period etc are set in Middleware,
    % so timestamp field is meaningless
    disp('Sending ready signal');
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(vehicle_id));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    readyStatusWriter.write(ready_msg);

    %% Wait for start or stop signal
    disp('Waiting for start or stop signal');
    systemTriggerReader.WaitSet = true;
    systemTriggerReader.WaitSetTimeout = 3600;
    got_stop = check_for_stop_signal(systemTriggerReader, trigger_stop);
    
    %% Run the HLC
    % Set reader properties
    systemTriggerReader.WaitSet = false;
    stateReader.WaitSet = true;
    stateReader.WaitSetTimeout = 60;
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
        disp('Waiting for data');
        [sample, ~, sample_count, ~] = stateReader.take();
        assert(sample_count == 1, 'Received %d samples, expected 1', sample_count);
        fprintf('Current time: %d\n',sample.t_now);
        
        if (reference_trajectory_time == 0)
            reference_trajectory_time = sample.t_now;
        end
        
        % Send the current trajectory point to the vehicle
        trajectory_point = TrajectoryPoint;
        trajectory_point.t.nanoseconds = uint64(reference_trajectory_time);
        trajectory_point.px = trajectory_px(reference_trajectory_index);
        trajectory_point.py = trajectory_py(reference_trajectory_index);
        trajectory_point.vx = trajectory_vx(reference_trajectory_index);
        trajectory_point.vy = trajectory_vy(reference_trajectory_index);
        vehicle_command_trajectory = VehicleCommandTrajectory;
        vehicle_command_trajectory.vehicle_id = uint8(vehicle_id);
        vehicle_command_trajectory.trajectory_points = trajectory_point;
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
                
        % Advance the reference state to T+2sec.
        % The reference state must be in the future,
        % to allow some time for the vehicle to receive
        % the message and anticipate the next turn.
        while (reference_trajectory_time < sample.t_now + 2000000000)
            reference_trajectory_time = ...
                reference_trajectory_time + segment_duration(reference_trajectory_index);
            reference_trajectory_index = ...
                mod(reference_trajectory_index, length(segment_duration)) + 1;
        end
                
        disp('Checking system trigger for stop signal');
        got_stop = check_for_stop_signal(systemTriggerReader, trigger_stop);
    end
end

function got_stop = check_for_stop_signal(systemTriggerReader, trigger_stop)
    [trigger, ~, sample_count, ~] = systemTriggerReader.take();
    got_stop = false;
    if sample_count > 0
        % look at most recent signal with (end)
        if trigger(end).next_start().nanoseconds() == trigger_stop
            got_stop = true;
        end
    end
end