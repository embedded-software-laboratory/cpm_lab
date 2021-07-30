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
function go_to_formation(vehicle_ids, p_goal)
    % vehicle_ids is a 1-by-nVeh array
    % p_goal is a 3-by-nVeh matrix with rows of x[m],y[m],yaw[deg]
    
    lab_domain_id = str2double(getenv("DDS_DOMAIN"));
    dt_max_delay = uint64(400e6); % Set correct middleware period
    
    % Initialize data readers/writers...
    % Change wd s.t. relative paths work
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    cd(script_directoy);
    % Import IDL files from cpm library
    dds_idl_matlab = fullfile('../../../cpm_lib/dds_idl_matlab/');
    assert(isfolder(dds_idl_matlab),...
        'Missing directory "%s".', dds_idl_matlab);
    assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
        'No MATLAB IDL-files found in %s', dds_idl_matlab);
    addpath(dds_idl_matlab)
    
    setenv("NDDS_QOS_PROFILES")
    
    %% variables for the communication
    topic_VehicleCommandTrajectory = 'vehicleCommandTrajectory';
    topic_systemTrigger = 'systemTrigger';
    topic_vehicleObservation = 'vehicleObservation';
    trigger_stop = uint64(18446744073709551615);

    %% create participant
    matlabParticipant = DDS.DomainParticipant('', lab_domain_id);
    %% create reader and writer
    % TODO first reader/writer creation takes ~25 seconds, accelerate?
    reader_systemTrigger = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'SystemTrigger', topic_systemTrigger);
    reader_vehicleObservation = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'VehicleObservation', topic_vehicleObservation);
    writer_vehicleCommandTrajectory = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandTrajectory', topic_VehicleCommandTrajectory);

    %% Run the HLC
    % Set reader properties
    reader_vehicleObservation.WaitSet = true;
    reader_vehicleObservation.WaitSetTimeout = 5;
    speed = 1; % [m/s]
    is_veh_in_formation = zeros(size(vehicle_ids));
    is_trajectory_planned = false;
    trajectory_points = TrajectoryPoint;
    if nargin<2
        goalPoses = homePosesFixed;
    else
        nVeh = numel(vehicle_ids);
        goalPoses = repmat(Pose2D, 1, nVeh);
        for iVeh = 1:nVeh
            goalPoses(iVeh).x = p_goal(1,iVeh);
            goalPoses(iVeh).y = p_goal(2,iVeh);
            goalPoses(iVeh).yaw = p_goal(3,iVeh);
        end
    end
    t_end = 0;
    active_vehicle_id = 0;
    got_stop = false;
    dt_loop = 0.3;
    while (~got_stop)
        t_loop_start = tic;
        % Stop if all vehicles are in formation
        if all(is_veh_in_formation) && ~is_trajectory_planned
            break;
        end
        % Read vehicle states
        [startPoses, t_now] = locate_all_vehicles(vehicle_ids, reader_vehicleObservation);

        %% If no trajectory planned, plan a new one
        if ~is_trajectory_planned
            
            %% Inner Loop fulfilling path planning task for each vehicle
            anyVehicleMovable = false;
            
            % find the first vehicle which is able to move and send trajectory to
            % corresponding vehicle.
            for iVeh = 1:nVeh
                if is_veh_in_formation(iVeh)
                    continue
                end
                active_vehicle_id = vehicle_ids(iVeh);
                fprintf("Starting to plan trajectory for vehicle %i.\n",active_vehicle_id);
                [trajectory_points, isPathValid] = planTrajectory( ...
                    vehicle_ids ...
                    ,startPoses ...
                    ,goalPoses(iVeh) ...
                    ,active_vehicle_id ...
                    ,speed ...
                );
                if isPathValid
                    anyVehicleMovable = true;
                    is_trajectory_planned = true;
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
                t_start = t_now + dt_max_delay;
                t_end = t_start + trajectory_points(end).t;
                is_t_start_init = true;
            end
            if t_now > t_end
                % trajectory is finished
                is_veh_in_formation(iVeh) = 1;
                is_trajectory_planned = false;
            else
                % send currently planned trajectory
                % adjust create and valid_after stamp
                trjMsg = trjMessage(trajectory_points, active_vehicle_id, t_start, t_now, dt_max_delay);
                writer_vehicleCommandTrajectory.write(trjMsg);
            end
        end
        if ~anyVehicleMovable
            disp("couldn't find a moveable vehicle. aborting.")
            break
        end

        [trigger, ~, sample_count, ~] = reader_systemTrigger.take();
        if sample_count > 0
            % look at most recent signal with (end)
            if trigger(end).next_start().nanoseconds() == trigger_stop
                got_stop = true;
                disp("go_to_formation: received stop signal, stopping.")
            end
        end
        
        dt_loop_actual = toc(t_loop_start);
        pause(dt_loop-dt_loop_actual);
    end
    
    disp('Finished');
end



function [poses, t_now] = locate_all_vehicles(vehicle_ids, reader)
    poses = struct('vehicle_id', 0, 'pose', []);
    maxVeh = 20;
    for iveh = maxVeh:-1:1
        poses(iveh).vehicle_id = 0;
    end
    veh_to_locate = zeros(maxVeh,1);
    veh_to_locate(vehicle_ids) = 1;

    while any(veh_to_locate)
        [sample, ~, ~, ~] = reader.take();
        for s = sample
            vid = s.vehicle_id;
            if vid == 0
                continue;
            else
                poseId = find(vehicle_ids==vid);
                poses(poseId).vehicle_id = vid;
                poses(poseId).pose = s.pose;
                poses(poseId).pose.yaw = s.pose.yaw * 180.0 / pi;
                t_now = s.header.valid_after_stamp.nanoseconds;
                veh_to_locate(vid) = 0;
            end
        end
    end

end