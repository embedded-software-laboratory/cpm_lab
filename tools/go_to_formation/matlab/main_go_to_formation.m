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
    dt_mw_nanos = uint64(400e6); % Set correct middleware period
    
    script_directoy = fileparts([mfilename('fullpath') '.m']);

    % Initialize data readers/writers...
    common_cpm_functions_path = fullfile( ...
        script_directoy ...
        ,".." ...
        ,".." ...
        ,".." ...
        ,"high_level_controller" ...
        ,"examples" ...
        ,"matlab" ...
    );
    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);

    % matlabParticipant needed in workspace
    [matlabParticipant, reader_vehicleStateList, writer_vehicleCommandTrajectory, ~, reader_systemTrigger, writer_readyStatus, trigger_stop] = init_script(matlabDomainID);

    
    % Send first ready signal 
    % Signal needs to be sent for all assigned vehicle ids
    % Also for simulated time case - period etc are set in Middleware, so timestamp field is meaningless
    vehicle_ids = varargin;
    disp('Sending ready signals');
    for i = 1 : length(vehicle_ids)
        ready_msg = ReadyStatus;
        ready_msg.source_id = strcat('hlc_', num2str(vehicle_ids{i}));
        ready_stamp = TimeStamp;
        ready_stamp.nanoseconds = uint64(0);
        ready_msg.next_start_stamp = ready_stamp;
        writer_readyStatus.write(ready_msg);
    end

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
    reader_vehicleStateList.WaitSetTimeout = 5;
    speed = 1; % [m/s]
    iPose = 1;
    vehicles_to_move = [vehicle_ids{:}];
    is_trajectory_planned = false;
    trajectory_points = TrajectoryPoint;
    goalPoses = homePosesFixed;
    t_end = 0;
    active_vehicle_id = 0;
    while (~got_stop)
        % Stop if all vehicles are in formation
        if isempty(vehicles_to_move) && ~is_trajectory_planned
            break;
        end

        % Read vehicle states
        sample = VehicleStateList; % Assign size, s.t. take() will only read one sample
        [sample, ~, ~, ~] = reader_vehicleStateList.take(sample);
        fprintf('Received sample at time: %d\n',sample.t_now);
        
        %% If no trajectory planned, plan a new one
        if ~is_trajectory_planned
            % Check if current sample contains data of all vehicles.
            % If not take new sample until data of all vehicles have been received.
            isSampleValid = true;
            for iVeh = 1:length(vehicle_ids)
                if sample.state_list(1,iVeh).vehicle_id == 0
                    isSampleValid = false;
                    break;
                end
            end
            if ~isSampleValid
                continue
            end
            
            % Set up static inputs for path planning task.
            startPoses = readPoses(sample.vehicle_observation_list);
            % extend to 20 poses for compatibility with codegen
            for iVeh = numel(startPoses)+1:20
                startPoses(iVeh).vehicle_id = 0;
                startPoses(iVeh).pose.x = 0; 
                startPoses(iVeh).pose.y = 0; 
                startPoses(iVeh).pose.yaw = 0;
            end
            % save('test_poses.mat', 'sample')
            
            %% Inner Loop fulfilling path planning task for each vehicle
            anyVehicleMovable = false;
            
            % find the first vehicle which is able to move and send trajectory to
            % corresponding vehicle.
            for iVeh = 1:length(vehicles_to_move)
                active_vehicle_id = vehicles_to_move(iVeh);
                fprintf("Starting to plan trajectory for vehicle %i.\n",active_vehicle_id);
                [trajectory_points, isPathValid] = planTrajectory( ...
                    vehicle_ids ...
                    ,startPoses ...
                    ,goalPoses(iPose) ...
                    ,active_vehicle_id ...
                    ,speed ...
                );
                if isPathValid
                    anyVehicleMovable = true;
                    is_trajectory_planned = true;
                    vehicles_to_move(iVeh) = [];
                    is_t_start_init = false;
                    fprintf("Successfully planned trajectory for vehicle %i.\n",active_vehicle_id);
                    break;
                else
                    % If planner returns invalid path continue with next vehicle.
                    fprintf("Could not plan valid trajectory for vehicle %i.\n",active_vehicle_id);
                end
            end
        else
        % trajectory is already planned, send until finish
            if ~is_t_start_init
                t_start = sample.t_now + dt_mw_nanos;
                t_end = t_start + trajectory_points(end).t;
                is_t_start_init = true;
            end
            if sample.t_now > t_end
                % trajectory is finished
                iPose = iPose + 1;
                is_trajectory_planned = false;
            else
                % send currently planned trajectory
                % adjust create and valid_after stamp
                trjMsg = trjMessage(trajectory_points, active_vehicle_id, t_start, sample.t_now, dt_mw_nanos);
                writer_vehicleCommandTrajectory.write(trjMsg);
            end
        end
        if ~anyVehicleMovable
            disp("couldn't find a moveable vehicle. aborting.")
            break
        end
        [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    end
    
    disp('Finished');
end