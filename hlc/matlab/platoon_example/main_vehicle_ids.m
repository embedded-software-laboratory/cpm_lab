function main(matlabDomainID, vehicleIDs)

    clc
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    cd(script_directoy)
    
    % Get all relevant files - IDL files for communication, XML files for communication settings etc
    git_directory = fileparts(fileparts(fileparts(script_directoy)));
    base_directory = fileparts(git_directory);
    cpm_idl_directory = [base_directory '/cpm_base/dds_idl'];
    hlc_idl_directory = [git_directory '/hlc/middleware/idl'];
    middleware_local_qos_xml = [git_directory '/hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml'];
    
    if ~exist(middleware_local_qos_xml,'file')
        error(['Missing middleware local QOS XML "' middleware_local_qos_xml '"'])
    end
    
    setenv("NDDS_QOS_PROFILES", ['file://' script_directoy '/QOS_READY_TRIGGER.xml;file://' middleware_local_qos_xml]);
    
    if ~exist(cpm_idl_directory, 'dir')
        error(['Missing directory "' cpm_idl_directory '"']);
    end
    if ~exist(hlc_idl_directory, 'dir')
        error(['Missing directory "' hlc_idl_directory '"']);
    end
    
    addpath(cpm_idl_directory);
    addpath(hlc_idl_directory);
    
    % Create .m IDL files in another directory, then go back to the current one
    mkdir('../IDL_gen');
    addpath('../IDL_gen');
    cd('../IDL_gen');

    DDS.import('VehicleStateList.idl','matlab', 'f')
    DDS.import('VehicleState.idl','matlab', 'f')
    DDS.import('VehicleCommandTrajectory.idl','matlab', 'f')
    DDS.import('SystemTrigger.idl','matlab','f')
    DDS.import('ReadyStatus.idl','matlab','f')
    
    cd(script_directoy)
    
    %% variables for the communication
    vehicle_ids = str2num(vehicleIDs);

    matlabStateTopicName = 'stateTopic';
    matlabCommandTopicName = 'trajectoryTopic';
    systemTriggerTopicName = 'system_trigger_hlc';
    readyStatusTopicName = 'ready_hlc';
    trigger_stop = uint64(18446744073709551615);

    phaseTime = 40;

    %% create participants
    matlabParticipant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlabDomainID);

    %% create reader and writer
    stateReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'VehicleStateList', matlabStateTopicName);
    trajectoryWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandTrajectory', matlabCommandTopicName);
    systemTriggerReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
    readyStatusWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

    %% wait for data if read() is used
    stateReader.WaitSet = true;
    stateReader.WaitSetTimeout = 10;

    %% Do not display figures
    set(0,'DefaultFigureVisible','off');

    % Send first ready signal 
    % Signal needs to be sent for all assigned vehicle ids
    % Also for simulated time case - period etc are set in Middleware, so timestamp field is meaningless
    disp('Sending ready signals');
    for i = 1 : length(vehicle_ids)
        ready_msg = ReadyStatus;
        ready_msg.source_id = strcat('hlc_', num2str(vehicle_ids(i)));
        ready_stamp = TimeStamp;
        ready_stamp.nanoseconds = uint64(0);
        ready_msg.next_start_stamp = ready_stamp;
        readyStatusWriter.write(ready_msg);
    end

    % Wait for start signal
    disp('Waiting for start or stop signal');
    got_stop = false;
    got_start = false;
    while(true)
        trigger = SystemTrigger;
        sampleCount = 0;
        [trigger, status, sampleCount, sampleInfo] = systemTriggerReader.take(trigger);
        while sampleCount > 0
            if trigger.next_start().nanoseconds() == trigger_stop
                got_stop = true;
            elseif trigger.next_start().nanoseconds() >= 0
                got_start = true;
            end
            [trigger, status, sampleCount, sampleInfo] = systemTriggerReader.take(trigger);
        end

        if got_stop
            disp("Got stop signal");
            break;
        elseif got_start
            disp("Got start signal");
            break;
        end
    end

    if got_stop == false
        while(true)
            disp('Checking system trigger for stop signal');
            trigger = SystemTrigger;
            sampleCount = 0;
            [trigger, status, sampleCount, sampleInfo] = systemTriggerReader.take(trigger);
            break_while = false;
            while sampleCount > 0
                current_time = trigger.next_start().nanoseconds();
                if current_time == trigger_stop
                    break_while = true;
                end
                [trigger, status, sampleCount, sampleInfo] = systemTriggerReader.take(trigger);
            end

            if break_while
                disp("Stopping bc of stop signal");
                break;
            end

            disp('Waiting for data');
            sample = VehicleStateList;
            status = 0;
            stateSampleCount = 0;
            sampleInfo = DDS.SampleInfo;
            [sample, status, stateSampleCount, sampleInfo] = stateReader.take(sample);

            % Check if any new message was received (TODO: throw away all messages that were received during computation)
            if stateSampleCount > 0                
                disp('Current time:');
                disp(sample.t_now);
            
                % Call the programs to calculate the trajectories of all HLCs
                if (size(vehicle_ids) > 0)
                    for i = 1 : length(vehicle_ids)
                        msg_leader = leader(vehicle_ids(i), sample.t_now);
                        trajectoryWriter.write(msg_leader);
                    end
                end
            end
        end
    end

    disp('Finished');

    trajectoryWriter.delete();
    stateReader.delete();
    matlabParticipant.delete();
    clear matlabParticipant;

end