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
function main_go_to_formation(varargin)
    matlabDomainID = 1;
    
    clc
%     script_directoy = fileparts([mfilename('fullpath') '.m']);
%     cd(script_directoy)
    
    % Initialize data readers/writers...
%     init_script_path = fullfile('../../../', '*', '/init_script.m');
    init_script_path = which('init_script.m');
    assert(isfile(init_script_path), 'Missing file "%s".', init_script_path);
%     addpath(fileparts(init_script_path));
    [matlabParticipant, stateReader, trajectoryWriter, systemTriggerReader, readyStatusWriter, trigger_stop] = init_script(matlabDomainID);
%     cd(script_directoy)

    vehicle_ids = varargin;
    vehicleList = {};
    
    for nVehicles = 1:length(vehicle_ids)
        vehicleList{nVehicles, 1} = strcat('vehicle_', num2str(vehicle_ids{nVehicles}));
    end
    
    speed = 0.75; % [m/s]
    isDriving = false; %any vehicle currently driving?
    
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
    %% 

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
            save('sample.mat', 'sample')
            
            for nVehicles = 1:length(vehicle_ids)
                stop_now = false;
                if sample.state_list(1,nVehicles).vehicle_id == 0
                   disp('Faulty sample. Stopping.')
                   stop_now = true;
                   break;
                end
            end
            
            if stop_now
                break;
            end
            
            startPoses = readPoses(sample.vehicle_observation_list);
            allHomePoses = homePosesFixed;
            goalPoses = startPoses;
            vehicleList = fields(startPoses);
            for nVehicles = 1:length(fields(goalPoses))
                goalPoses.(vehicleList{nVehicles}) = allHomePoses(nVehicles);
            end
            
            isCostMapShown = true;
            isOccMapShown = true;
            
            for nVehicles = 1:length(vehicleList)
                                
                startPoses = readPoses(sample.vehicle_observation_list);
                disp(startPoses)
                
                egoVehicle = vehicleList{nVehicles}; %TODO advanced detection of ego vehicle
               
                map = setOccMap(startPoses, egoVehicle);
                if isOccMapShown == false
                    show(map)
                    isOccMapShown = true;
                end
                [refPath, Fig] = PlanAndShowRRTPath(startPoses, goalPoses, egoVehicle, map);
                
                %Show occupancy grid and path when planner executed
                %for the first time
                if isCostMapShown == false
                    figure(Fig)
                    isCostMapShown = true;
                end
%                 
                trajectory_points = pathToTrajectory(refPath, speed);
                
    %           
    %             % Check if any new message was received (TODO: throw away all messages that were received during computation)
    %             if stateSampleCount > 0                
    %                 disp('Current time:');
    %                 disp(sample.t_now);

                 [sample, status, stateSampleCount, sampleInfo] = stateReader.take(sample);
                 save('sample.mat', 'sample')
                 t_start = sample.t_now;
                 trjMsg = trajMessage(trajectory_points, vehicle_ids{nVehicles}, t_start, sample.t_now);
                 trajectoryWriter.write(trjMsg);
                 
                 isDriving = true;
                 stop_now = false;
                 t_now = t_start;
                 
                 while(t_now <= t_start + trajectory_points(end).t)
                     
                    disp('Checking system trigger for stop signal');
                    [trigger, status, sampleCount, sampleInfo] = systemTriggerReader.take(trigger);
                    current_time = trigger.next_start().nanoseconds();
                    if current_time == trigger_stop
                        disp("Stopping");
                        stop_now = true;
                        break;
                    end
                                                    
                    [sample, status, stateSampleCount, sampleInfo] = stateReader.take(sample);
                    t_now = sample.t_now;
%                     currentPoses = readPoses(sample.vehicle_observation_list);
%                     map = setOccMap(currentPoses, egoVehicle);
                    occupied = checkOccupancy(map, [goalPoses.(vehicleList{nVehicles}).x goalPoses.(vehicleList{nVehicles}).y]);
                    disp(occupied)
                   
%                     if checkOccupancy(map, [goalPoses.(vehicleList{nVehicles}).x goalPoses.(vehicleList{nVehicles}).y]) == 1
%                         isDriving = false;
%                         break;
%                     end
                    trjMsg = trajMessage(trajectory_points, vehicle_ids{nVehicles}, t_start, sample.t_now);
                    trajectoryWriter.write(trjMsg);
                    
                 end
                   
               if stop_now
                   break;
               end
               
            end
            
            if stop_now
                break;
            end
            
        end
     end
             
            
       
     %% 
 

    disp('Finished');

    trajectoryWriter.delete();
    stateReader.delete();
    matlabParticipant.delete();
    clear matlabParticipant;

end