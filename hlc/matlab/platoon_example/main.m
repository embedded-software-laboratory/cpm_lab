function main(matlabDomainID, vehicleIDs)
    %Important: Call with period of 250ms
    %setenv("NDDS_QOS_PROFILES", "file:///home/controller/Documents/GIT/software/Middleware/build/QOS_LOCAL_COMMUNICATION.xml");
    
    clc
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    cd(script_directoy)
    setenv("NDDS_QOS_PROFILES", ['file://' script_directoy '/QOS_READY_TRIGGER.xml;file://' script_directoy '/QOS_LOCAL_COMMUNICATION.xml']);
    
    
    %% Include IDL path
    addpath('./IDL');
    mkdir('./IDL_gen');
    cd('./IDL_gen');

    DDS.import('VehicleStateList.idl','matlab', 'f')
    DDS.import('VehicleState.idl','matlab', 'f')
    DDS.import('VehicleCommandTrajectory.idl','matlab', 'f')
    DDS.import('SystemTrigger.idl','matlab','f')
    DDS.import('ReadyStatus.idl','matlab','f')
    
    cd ..
    addpath('./IDL_gen');
    
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
        ready_msg.source_id = num2str(vehicle_ids(i));
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

            % Check if any new message was received (TODO: History of 1)
            if stateSampleCount > 0                
                disp('Current time:');
                disp(sample.t_now);
            
                % Call the programs to calculate the trajectories of all HLCs
                if (size(vehicle_ids) > 0)
                    msg_leader = dummy_trajectory_complex_part(vehicle_ids(1), sample.t_now);
                    trajectoryWriter.write(msg_leader);

                    for i = 2 : length(vehicle_ids)
                        msg_follower = dummy_following_part(vehicle_ids(i), sample.state_list, vehicle_ids(i - 1), sample.t_now);
                        trajectoryWriter.write(msg_follower);
                    end
                end
            end
        end
    end

    %% w = waitforbuttonpress
    %% end

    disp('Finished');

    trajectoryWriter.delete();
    stateReader.delete();
    matlabParticipant.delete();
    clear matlabParticipant;

end