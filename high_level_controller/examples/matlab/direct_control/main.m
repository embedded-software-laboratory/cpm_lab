function main(vehicle_id)    
    matlabDomainID = 1;
    
    % Import IDL files from cpm library
    dds_idl_matlab = fullfile('../../../cpm_lib/dds_idl_matlab/');
    if ~exist(dds_idl_matlab, 'dir')
        error(['Missing directory "' dds_idl_matlab '"']);
    end
    addpath(dds_idl_matlab)

    % XML files for quality of service settings
    middleware_local_qos_xml = '../../../middleware/build/QOS_LOCAL_COMMUNICATION.xml';
    if ~exist(middleware_local_qos_xml,'file')
        error(['Missing middleware local QOS XML "' middleware_local_qos_xml '"'])
    end

    ready_trigger_qos_xml = './QOS_READY_TRIGGER.xml';
    if ~exist(ready_trigger_qos_xml,'file')
        error(['Missing ready trigger QOS XML "' ready_trigger_qos_xml '"'])
    end
    
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

    topic_vehicleCommandDirect = 'vehicleCommandDirect';
    writer_vehicleCommandDirect = DDS.DataWriter(...
        DDS.Publisher(matlabParticipant),...
        'VehicleCommandDirect',...
        topic_vehicleCommandDirect);
    
    
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
    
    while (~got_stop)
        % Read vehicle states
        [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
        assert(sample_count == 1, 'Received %d samples, expected 1', sample_count);
        fprintf('Received sample at time: %d\n',sample.t_now);
        
        % Determine control inputs for the vehicle
        % right curve with moderate forward speed
        vehicle_command_direct = VehicleCommandDirect;
        vehicle_command_direct.header.create_stamp.nanoseconds = uint64(sample.t_now);
        vehicle_command_direct.header.valid_after_stamp.nanoseconds = uint64(sample.t_now);
        vehicle_command_direct.vehicle_id = uint8(vehicle_id);
        vehicle_command_direct.motor_throttle =  0.3;
        vehicle_command_direct.steering_servo = -0.45;
        
        writer_vehicleCommandDirect.write(vehicle_command_direct);
                
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