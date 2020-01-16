function main_vehicle_ids(matlabDomainID, varargin)
    clc
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    cd(script_directoy)
    
    % Initialize data readers/writers...
    init_path = fullfile('./');
    if ~exist("./init_script.m", 'file')
        error(['Missing file "' init_path '"/init_script.m']);
    end
    addpath(init_path)
    [matlabParticipant, stateReader, trajectoryWriter, systemTriggerReader, readyStatusWriter, trigger_stop] = init_script(matlabDomainID);
    cd(script_directoy)

    vehicle_ids = varargin

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
        ready_msg.source_id = strcat('hlc_', num2str(vehicle_ids{i}));
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
                        msg_leader = leader(vehicle_ids{i}, sample.t_now);
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