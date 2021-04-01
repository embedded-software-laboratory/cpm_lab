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

classdef  PathPlanner < handle
% path planner based on lanelet data, each vehicle owns one PathPlanner    
% path are always sampling points of the centerline
%% -----------------------------------------------------------
%  
%       Constructor
%               obj=PathPlanner(int_vehicle_id,ext_vehicle_id)
%       function
%          obj=updateRandomPath(obj,r_tree,map,pos):
%                   update planned path and path lanelets by extending current trajectory
%          obj = initializeRandomPathFromStart(obj,r_tree,map,pos):
%                   reset planned path from new starting point


%% -----------------------------------------------------------
    properties (Access=public)
        path_pos;             % [px1,...,pxn; py1,...,pyn]
        path_speed;             % maximum velocity at sampling point.
        ext_vehicle_id ;             % id of this vehicle
        int_vehicle_id ;             % internal id between 1 and numVhhicle 
        current_lanelet_id;      % id of current lanelet
        next_lanelet_ids;          % id of next lanelet in path schedule 
        next_edge_index = int32(0); % edge index in current lanelet
        % array of id's for next lanelets.
        numLanelets_planned = uint8(15); % length of path-lanelet queue
        projected_pose;     % projected position on centerline

    end
    
    methods
        function obj=PathPlanner(int_vehicle_id,ext_vehicle_id)
             obj.int_vehicle_id = int_vehicle_id;
             obj.ext_vehicle_id = ext_vehicle_id;
             obj.next_lanelet_ids = int32(zeros(1,obj.numLanelets_planned));
        end
        %% randomized central routing behaviour
        function obj=updateRandomPath(obj,r_tree,map,pos)
            
            %% get current lanelet
            Lanelet_ids = r_tree.searchForLanelets(pos);
            if isempty(Lanelet_ids)
                % no candidates, position not in map. 
                error('position not found in updateRandomPath');
            else
                if ismember(obj.current_lanelet_id,Lanelet_ids)
                    % obj.next_lanelet_ids remains the same, only adjust
                    % path by edge number
                    
                    Iterator = map.Search(obj.current_lanelet_id).value;
                    [next_edge, ~] = Iterator.find_next_edge(pos);
                    obj.projected_pose = Iterator.projectionOnCenterline(pos);
                    
                    offset = max(0,next_edge-obj.next_edge_index);

                    
                    % compute new centerline
                    centerline_comp = obj.path_pos(:,1+offset:end);
                    speed_allowed = obj.path_speed(:,1+offset:end);
                    obj.next_edge_index = next_edge;
                    
                     obj.path_pos = centerline_comp;
                     obj.path_speed = speed_allowed;
                else
                    % current lanelet id changed
                    foundflag = int8(0);
                    idxLastScheduledLanelet = 0; % index of last lanelet in schedule.
                    for k=1:obj.numLanelets_planned
                        if ismember(obj.next_lanelet_ids(k),Lanelet_ids) % vehicle moved to successor lanelet

                            idxLastScheduledLanelet=obj.numLanelets_planned-k;
                            % shift plans by k
                            obj.current_lanelet_id = obj.next_lanelet_ids(k);
                            obj.next_lanelet_ids(1:idxLastScheduledLanelet) = obj.next_lanelet_ids(1+k:obj.numLanelets_planned);

                            Iterator = map.Search(obj.current_lanelet_id).value;

                            % current lanelet, needs to consider progress
                            [next_edge, ~] = Iterator.find_next_edge(pos);
                            obj.projected_pose = Iterator.projectionOnCenterline(pos);
                            
                            foundflag=1;
                            break;
                        end
                    end
                    
                    if foundflag == 0
                        Lanelet_ids
                        error('TrajectoryPlanner:position not in target lanelets, return');
                    end
                    centerline = Iterator.get_centerline();
                    speedProfile = Iterator.get_speedProfile();               
                    
                    
                    
                    % pos
                    centerline_comp = centerline(:,next_edge:end); % skip the edges we already passed
                    speedProfile_comp = speedProfile(next_edge:end);
                    obj.next_edge_index = next_edge;
                    
                    % any successing lanelet can be fully concatenated
                    for k = 1:idxLastScheduledLanelet
                        next_id = obj.next_lanelet_ids(k);
                        Iterator  = map.Search(next_id).value;
                        
                        centerline_comp = [centerline_comp(:,1:end-1), Iterator.get_centerline() ];
                        speedProfile_comp = [speedProfile_comp(1:end-1), Iterator.get_speedProfile() ];
                        
                    end
                    
                    for k=idxLastScheduledLanelet : obj.numLanelets_planned-1
                        % last one may be dead end, so check if hasSucc==1
                        
                        Iterator = map.Search(obj.next_lanelet_ids(k)).value;
                        [next_id, ~] = Iterator.choose_random_successor();
                        obj.next_lanelet_ids(k+1) = next_id;
                        
                        
                        Iterator  = map.Search(next_id).value;
                        
                        % pos
                        centerline_comp = [centerline_comp(:,1:end-1), Iterator.get_centerline() ];
                        speedProfile_comp = [speedProfile_comp(1:end-1), Iterator.get_speedProfile() ];
                        
                    end
                    
                    obj.path_pos = centerline_comp;
                    obj.path_speed = speedProfile_comp;
                end
            end
             
        end
            
        function obj = initializeRandomPathFromStart(obj,r_tree,map,pos)
                % generates the initial path for the central routing
                % problem
            
                Lanelet_ids = r_tree.searchForLanelets(pos);
                if isempty(Lanelet_ids)
                    msg = ['cannot match x=' ,num2str(pos(1)),' ; y=',num2str(pos(2)),' of vehicle ',num2str(obj.ext_vehicle_id)];
                    error(msg);    
                end 
                obj.current_lanelet_id = int32(Lanelet_ids(1));
                currLanelet = map.Search(obj.current_lanelet_id ).value;
                Iterator = currLanelet;
                [next_edge, ~] = Iterator.find_next_edge(pos); 

                
                centerline = Iterator.get_centerline();
                speedProfile = Iterator.get_speedProfile();
                centerline_comp = centerline(:,next_edge:end);
                speedProfile_comp = speedProfile(1,next_edge:end);
                
                for k = 1:obj.numLanelets_planned
                    [next_id, ~]=Iterator.choose_random_successor();
                    
                    obj.next_lanelet_ids(k) = next_id;
                    Iterator = map.Search(next_id).value;
                    
                    centerline_comp = [centerline_comp(:,1:end-1), Iterator.get_centerline()]; % end point and start point overlap, so delete end point
                    speedProfile_comp = [speedProfile_comp(:,1:end-1), Iterator.get_speedProfile()]; % end point and start point overlap, so delete end point
                    
                end
                obj.next_edge_index = next_edge;
                obj.path_pos = centerline_comp;
                obj.path_speed = speedProfile_comp;
                
        end
        
       
    end
    
end
