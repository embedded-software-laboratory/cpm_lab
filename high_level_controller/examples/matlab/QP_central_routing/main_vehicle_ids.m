function main_vehicle_ids(varargin)
    matlabDomainID = 1;
    clc
    
    
    vehicle_ids = varargin
    numVehicles = length(vehicle_ids); 
    
%% read paths
    script_directory = fileparts([mfilename('fullpath') '.m']);
    cd(script_directory)
    
    % Initialize data readers/writers...
    init_script_path = fullfile('../', '/init_script.m');
    assert(isfile(init_script_path), 'Missing file "%s".', init_script_path);
    addpath(fileparts(init_script_path));
    % CAVE `matlabParticipant`must be stored for RTI DDS somewhere
    %   in the workspace  (so it doesn't get gc'ed)
    [matlabParticipant, stateReader, trajectoryWriter, ~, systemTriggerReader, readyStatusWriter, trigger_stop] = init_script(matlabDomainID);
    cd(script_directory)


    
    %% IBM CPLEX path
    CPLEX_PATH = '/opt/ibm/ILOG/CPLEX_Studio1210/cplex/matlab/x86-64_linux';
    addpath(CPLEX_PATH);
    
    
    %% import CommonRoad path
    addpath('./RTree');
    addpath('./Maps');
    
    %% HLC 
    addpath('./ControllerFiles');
    
   

    %% initialisation procedure
    filepath = './Maps/LabMapCommonRoad.xml'; 
    commonroad_data = LoadXML(filepath);

    
   

    
    %% initialize vehicles 
    
    StartLaneletId = 0*(1:numVehicles); % dummy zeros, because GUI initialize vehicles somewhere random
    
    % flags for Initialization of each vehicle
    VehNotStarted = ones(numVehicles,1);
    activeVehicleList = [];
    bool_isOfflinePhase = uint8(1); % flag for synchronized start
    
    vehicles = cell(1,numVehicles);

    for k = 1:numVehicles
        vehicles{1,k} = vehicle(k,vehicle_ids{k},StartLaneletId(k),commonroad_data.map,commonroad_data.r_tree);
    end
 
    % storage of trajectory data of each vehicle, passed to hlc for collision avoidance
    hlc_trajectoryLastIter = cell(numVehicles,2);
    
    %% initalize DDS 
    
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
    
   
 %% enter loop
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
                
         
                % read status to see if position is delivered by middleware, assuming
                % vehicle not starting at (0,0) (initialized with (0,0) by
                % default)
                %if active, initialize hlc of vehicles. 
                if bool_isOfflinePhase == 1
                    for k = 1:numVehicles
                        if VehNotStarted(k) == 1
                                msg = ['sample.state_list(1,',num2str(k),'): x=',num2str(sample.state_list(1,k).pose.x),'; y= ',num2str(sample.state_list(1,k).pose.y)];
                                disp(msg);
                                msg = ['sample.state_list(1,',num2str(k),'): speed= ',num2str(sample.state_list(1,k).speed)];
                                disp(msg);
                                msg = ['sample.state_list(1,',num2str(k),'): yaw= ',num2str(sample.state_list(1,k).pose.yaw)];
                                disp(msg);
                                
                                % vehicle is active
                                if ~(sample.state_list(1,k).pose.x == 0 && sample.state_list(1,k).pose.y == 0)
                                    VehNotStarted(k) = 0; % vehicle is online
                                    activeVehicleList = sort([activeVehicleList, k]);         
                                end
                        end
                    end
                    
                    % check if all vehicles are active. if yes, go to
                    % online phase
                    if sum(VehNotStarted) == 0
                        bool_isOfflinePhase =0; % go online
                        % initialize path planning
                        for k = 1:numVehicles
                            vehicles{1,k}.update(sample.state_list(1,k));
                            vehicles{1,k}.resetPathPlanner();
                        end
                    else
                        % not all vehicles are online, skip current loop
                        % iteration until all vehicles are online
                        continue;
                    end
                end

                

                
                
                % Call the programs to calculate the trajectories of all HLCs
                if (size(vehicle_ids) > 0)
                     
                       for j = 1 : numVehicles
                           
                            k = numVehicles +1 - j;
                            if VehNotStarted(k) == 0
                                % position update and ref traj update
                                vehicles{1,k}.update(sample.state_list(1,k)); % position update
                                vehicles{1,k}.updatePath(sample.t_now); % path update
                                
                                
                                % get active vehicles with higher priority
                                higherPriorityindex = (activeVehicleList>k);
                                higherPriorityVehicles = activeVehicleList(higherPriorityindex);     
                                VehicleCollision =  hlc_trajectoryLastIter(higherPriorityVehicles,:);
                                
                                % no obstacles in cenral routing, only
                                % Vehicles
                                dynamicObstaclesVehicles = VehicleCollision;
                                
                                
                                % calculate trajectory
                                vehicles{k}.optimizeTraj(dynamicObstaclesVehicles);
                                
                                % convert to message
                                [msg, position, time ]= vehicles{1,k}.getTrajMsg();
                                trajectoryWriter.write(msg);
                                
                                % update collision trajectory.
                                hlc_trajectoryLastIter{k,1} = position;  % update;
                                hlc_trajectoryLastIter{k,2} = time;  % update;
                            end
                            
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
