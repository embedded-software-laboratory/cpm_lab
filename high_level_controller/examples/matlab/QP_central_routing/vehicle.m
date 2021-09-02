classdef vehicle < handle
%% ----------------------------------------------------------- 
%     Constructor
%             obj=vehicle(int_vehicle_id,ext_vehicle_id,StartLaneletId,map,r_tree) 

%     function
%      obj=update(obj,state_list):
%                   reads states from state_list
%      obj = updatePath(obj,t_now):
%                   updates planned/remaining path for new timestep
%      resetPathPlanner(obj)
%                   generates new path from current Position
%      obj=optimizeTraj(obj,dynamicObstacles) :
%                   calculate trajectory and save results in this object
%       [msg, position, time ]= getTrajMsg(obj): 
%                   creates the message for the VehicleTrajectoryCommand in
%                   msg. also returns trajectory data by position and time


%% -----------------------------------------------------------
    properties (Access = public)
       ext_vehicle_id ;             % id of this vehicle
       int_vehicle_id ;             % internal id, mapped between 1 and numVehicle 
       current_lanelet_id;      % id of current lanelet
       
       map;             % map (data structure) of road map.
       r_tree;
       
       pose;            % current position 
       speed;           %  [m/s]
       maxSpeed = 2;           % limitation : max speed of vehicle [m/s]
       PathPlanner;     
       bool_isFirstIter = int8(1); % flag

       
      % integer value, delay for consistency with mid level controller, see thesis
       control_delay = 1+1; 
       
       t_eval; % "k=0"-time in nanosec for controller in current time step
       
       du;      % delta u, hlc input vector
       Hu; 
       Hp; 
       dt;  % [sec]
       speed_target ; %[m/s]
       dsafeVehicles ; %[m]
       model; % vehicle model
       minSpeed; % [m/s]
       
       % Past trajectory data for traj_msg
       lastTraj = []; % [x;y;vx;vy;speed]
       lastTime ; % separate, because other data type
       idx_pos = 1:2;
       idx_vel = 3:4;
       idx_speed = 5;
       du_prev;
       speed_eval % speed at t_eval
      
    end
    
    methods
        
        %% cunstructor, initialize vehicle states
        function obj=vehicle(int_vehicle_id,ext_vehicle_id,StartLaneletId,map,r_tree) 
            
            obj.int_vehicle_id=uint8(int_vehicle_id);
            obj.ext_vehicle_id=uint8(ext_vehicle_id);            
            obj.map = map;
            obj.r_tree =r_tree;
%             obj.current_lanelet_id = int16(StartLaneletId);
            obj.speed= obj.minSpeed;
            obj.PathPlanner=PathPlanner(int_vehicle_id,ext_vehicle_id);
            
            model = IntegratorModel();
            obj.model = model;
             
             
             % load configuration
             cfg = configVehicle();
             obj.Hu = cfg.Hu;
             obj.Hp = cfg.Hp;
             obj.dt = cfg.dt;
             obj.minSpeed = cfg.minSpeed;
             obj.dsafeVehicles = cfg.dsafeVehicles;
             obj.speed_target = cfg.speed_target;
             obj.du = zeros(model.nu,obj.Hu);
        end
       
        function obj=update(obj,state_list)
            
            obj.pose = state_list.pose; 
            obj.speed = max(state_list.speed,obj.minSpeed);
        end
        
        % update path dynamically by removing control points already passed
        function obj = updatePath(obj,t_now)
            
            dt_nanos = uint64(obj.dt*1e9);
            t_now_sample = ((t_now ) / dt_nanos ) * dt_nanos ;   
            %% update trajectory planner
            % plan for pos(k=0) and pos(k=1) are already fixed 
            % pos(k=2) also fixed because of central difference tangents
            if obj.bool_isFirstIter % first iteration 
                pos_eval = [obj.pose.x;obj.pose.y];
                obj.t_eval = t_now_sample+ obj.control_delay*dt_nanos;
                obj.speed_eval = obj.minSpeed;
            else % default case
                obj.t_eval = t_now_sample +obj.control_delay*dt_nanos;
                idx_teval = find(obj.lastTime>= obj.t_eval,1);
                pos_eval = obj.lastTraj(obj.idx_pos,idx_teval);
                obj.speed_eval =  obj.lastTraj(obj.idx_speed,idx_teval);
            end
            obj.PathPlanner.updateRandomPath(obj.r_tree,obj.map,pos_eval);
           

        end
        
        % resets the scheduled path
        function resetPathPlanner(obj)
            pos_eval = [obj.pose.x;obj.pose.y];
            obj.PathPlanner.initializeRandomPathFromStart(obj.r_tree,obj.map,pos_eval);
           
        end
        
        % trajectory generation by MPC
        function obj=optimizeTraj(obj,dynamicObstacles) 
            
            % delay already considered in path
            path_pos = obj.PathPlanner.path_pos; 
            path_speed = obj.PathPlanner.path_speed;
            
            if size(path_pos,2) > 0 % if == 0, then we are at goal state and dont need to update anymore
                mpc_info = struct;
                mpc_info.dsafeVehicles = obj.dsafeVehicles;
                
                mpc_info.Hp = obj.Hp;
                mpc_info.Hu = obj.Hu;
                
                nObst = size(dynamicObstacles,1);
                mpc_info.nObst = nObst;
                mpc_info.dt = obj.dt;
                vehMaxSpeed = obj.maxSpeed ;
                speedTarget = obj.speed_target;
                

                speed_ref = min(min(path_speed,vehMaxSpeed), speedTarget); 
                
                mpc_info.path_speed = speed_ref;
                mpc_info.path_pos = path_pos;
                mpc_info.du_prev = obj.du;
                mpc_info.dynamicObstacles = dynamicObstacles; % trajectory of collision
                mpc_info.model = obj.model;
                mpc_info.minSpeed=obj.minSpeed;
                
                
                curr_state = struct;
                curr_state.pos =obj.PathPlanner.projected_pose;% obj.checkpoint_pos;
                
                curr_state.speed = obj.speed_eval; 
                
                %% call MPC to optimize
                disp('entering MPC');
                [trajPredPos,trajPredSpeed, du_result,feasible] = Priority_QP(curr_state,mpc_info);
                if ~feasible
                    disp(['infeasible vehicle ',num2str(obj.ext_vehicle_id) ]);
                end
                disp('MPC finished');
                
                %% postprocess MPC results
                % we need tangents at control points, which we get using
                % central difference
                % for this, we also need to be consistent to the data from the last
                % mpc cycle for pos(k=0,1,2)
                %
                % time stamps are stored seperately from trajectory because
                % the data type is different
                
                 dt_nanos = uint64(obj.dt*1e9);
                time = (obj.t_eval) : (dt_nanos) : (obj.t_eval + obj.Hp * dt_nanos);
                eps = 1e-16; % avoid division with zero

                if obj.bool_isFirstIter
                    newTrajectory(obj.idx_pos,:) = [curr_state.pos, trajPredPos];
                    newTime= uint64(time);
                    newTrajectory(obj.idx_speed,:) = [curr_state.speed, trajPredSpeed];
                    obj.du_prev = du_result;
                    
                    % velocity needs tangents
                    diff_point = newTrajectory(obj.idx_pos,2:end)-newTrajectory(obj.idx_pos,1:end-1) ; % difference between points
                    central_diff = 0.5*  diff_point(:,1:end-1) + 0.5*  diff_point(:,2:end); % central difference
                    central_diff_normed = central_diff ./ sqrt(central_diff(1,:).^2 + central_diff(2,:).^2 +eps); % normalize
                    tangent_speed = max(obj.minSpeed, trajPredSpeed);
                    firstTangent = tangent_speed(1)*diff_point(:,1)/norm(diff_point(:,1)+eps);
                    lastTangent = tangent_speed(end)*diff_point(:,end)/norm(diff_point(:,1)+eps);
                    newTrajectory(obj.idx_vel,:) = [firstTangent, newTrajectory(obj.idx_speed,2:end-1).*central_diff_normed,lastTangent];
                    obj.bool_isFirstIter = 0;
                    cutOffIndex = 1; % 1= no cutOff
                else
                    % update trajectory: substitute corresponding block with new
                    % trajectory
                    % time(2) = uint64(obj.t_eval + obj.dt*1e9)
                    idx_update = find(obj.lastTime>= time(2)-1,1); % index of t_eval+dt column
                    numPastPoints = 0;
                    cutOffIndex= max(1, idx_update-1-obj.control_delay-numPastPoints);
                    idx_prev = 1:idx_update-1; % from 1 to t_eval
                    
                    
                    newTrajectory(obj.idx_pos,:) = [obj.lastTraj(obj.idx_pos,idx_prev),trajPredPos];
                    newTime= [obj.lastTime(:,idx_prev),time(2:end)];
                    newTrajectory(obj.idx_speed,:) = [obj.lastTraj(obj.idx_speed,idx_prev),trajPredSpeed];
                    obj.du_prev = du_result;
                    
                    % velocity needs tangents
                    pos_to_diff = [ newTrajectory(obj.idx_pos,idx_update-2:end)];
                    diff_point = pos_to_diff(:,2:end)-pos_to_diff(:,1:end-1) ; % difference between points
                    central_diff = 0.5*  diff_point(:,1:end-1) + 0.5*  diff_point(:,2:end); % central difference
                    % normalized tangents for x(t_eval),..,x(t_eval+Hp-1)
                    central_diff_normed = central_diff ./ sqrt(central_diff(1,:).^2 + central_diff(2,:).^2 +eps); % normalize
                    
                    
                    % non zero speed necessary, beginning from t_eval to t_eval+Hp
                    tangent_speed = max(obj.minSpeed, newTrajectory(obj.idx_speed,idx_update-1:end));
                    % last point with backward difference
                    lastTangent = tangent_speed(end)*diff_point(:,end)/(norm(diff_point(:,end))+eps);
                    
                    
                    % copy old tangents
                    newTrajectory(obj.idx_vel,idx_prev) = obj.lastTraj(obj.idx_vel,idx_prev);
                    % new tangents. idx_update-1 has to be overwritten as well
                    % because the tangents changes.
                    newTrajectory(obj.idx_vel,idx_update-1:end) = [tangent_speed(1:end-1).*central_diff_normed(),lastTangent];
                end
                
                % update trajectory plans
                obj.lastTraj = newTrajectory(:,cutOffIndex:end);
                obj.lastTime = newTime(cutOffIndex:end);
                
            end
        end
        function [msg, position, time ]= getTrajMsg(obj)
            
            %% convert to message
                      
             
            % Create msg for VehicleTrajCommand
            trajectory = VehicleCommandTrajectory;
            trajectory.vehicle_id = uint8(obj.ext_vehicle_id);
            trajectory.header.create_stamp.nanoseconds = obj.t_eval;
            trajectory.header.valid_after_stamp.nanoseconds = obj.t_eval ;    
            trajectory_points = [];
            nTimePoints = size(obj.lastTraj,2);
           for k = 1: nTimePoints
                % Get current trajectory from pre-computed trajectory list
                trajectory_point =  obj.lastTraj([obj.idx_pos obj.idx_vel ],k);
                
                point1 = TrajectoryPoint;
                  
                stamp = TimeStamp;
                stamp.nanoseconds = uint64(obj.lastTime(k));
                point1.t = stamp;
                point1.px = trajectory_point(1);
                point1.py = trajectory_point(2);
                point1.vx = trajectory_point(3);
                point1.vy = trajectory_point(4);
                
                trajectory_points = [trajectory_points [point1]];
           end
  
            trajectory.trajectory_points = trajectory_points;
            % Return msg
            msg = trajectory;
            position = obj.lastTraj(obj.idx_pos,:);
            time = obj.lastTime;
        end       
        
    end
    
    
    
end
