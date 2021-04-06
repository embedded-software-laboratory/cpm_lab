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

classdef PathPlanner < handle
% path planner based on lanelet data, each vehicle owns one PathPlanner    
% path are always sampling points of the centerline
%% -----------------------------------------------------------
%  
%       Constructor
%               obj=PathPlanner(int_vehicle_id,ext_vehicle_id)
%       function
%          obj = updatePath(obj,r_tree,map,pos):
%                   update planned path and path lanelets by removing path
%                   points we already visited
%          obj = SearchPathToGoal(obj,r_tree,map,pos,planning_target_k,succGraph):
%                   updates path to goal from new starting point


%% -----------------------------------------------------------
    properties (Access=public)
        path_pos;             % [px1,...,pxn; py1,...,pyn]
        path_speed;             % maximum velocity at sampling point.
        ext_vehicle_id ;             % id of this vehicle
        int_vehicle_id ;             % internal id between 1 and numVhhicle 
        current_lanelet_id;      %id of current lanelet
        next_lanelet_ids;
        next_edge_index = int32(0);
        % array of id's for next lanelets.
        numLanelets_planned = uint8(6);
        projected_pose;

    end
    
    methods
        function obj=PathPlanner(int_vehicle_id,ext_vehicle_id)
             obj.int_vehicle_id = int_vehicle_id;
             obj.ext_vehicle_id = ext_vehicle_id;
             obj.next_lanelet_ids = int32(zeros(1,obj.numLanelets_planned));
        end
        
        %% goal planning problem
        function obj=updatePath(obj,r_tree,map,pos)
            
            %% get current lanelet
            Lanelet_ids = r_tree.searchForLanelets(pos);
            if isempty(Lanelet_ids)
                error('cannot match position in updatePath');
            else
                if ismember(obj.current_lanelet_id,Lanelet_ids)
                    % current_lanelet_id and next_lanelet_ids remain the same, only adjust
                    % path by edge number
                    Iterator = map.Search(obj.current_lanelet_id).value;
                    
                else
                    
                    foundflag = int8(0);
                    numLanelets = length(obj.next_lanelet_ids);               
                    % search path for current lanelet to check if still on path
                    for k=1:numLanelets
                        if ismember(obj.next_lanelet_ids(k),Lanelet_ids) % vehicle moved to successor lanelet
                            
                            % shift plans by k
                            obj.current_lanelet_id = obj.next_lanelet_ids(k);
                            obj.next_lanelet_ids = obj.next_lanelet_ids( (1+k) : end); % shrink
                            
                            Iterator = map.Search(obj.current_lanelet_id).value;
                                   
                            foundflag=1;
                            break;
                        end
                    end
                    if foundflag == 0
                        % do nothing and return;
                        disp('position not in path, return');
                        return;
                    end
                end
                [next_edge, ~] = Iterator.find_next_edge(pos); 
                obj.projected_pose = Iterator.projectionOnCenterline(pos);
                centerline = Iterator.get_centerline();
                next_proj_pos = centerline(:,next_edge); % skip the edges we already passed
                
                eps = 1e-4; % tolerance
                dist_x = (obj.path_pos(1,:) - next_proj_pos(1)).^2;
                dist_y = (obj.path_pos(2,:) - next_proj_pos(2)).^2;
                index_vector_x = find( dist_x< eps);
                index_vector_y = find( dist_y< eps);
                index_next = intersect(index_vector_x,index_vector_y);

                index_next = index_next(end);
                centerline_comp =  obj.path_pos(:,index_next:end);
                speedProfile_comp =  obj.path_speed(:,index_next:end);
                obj.path_pos = centerline_comp;
                obj.path_speed = speedProfile_comp;
                
                   
            end
             
        end
         
        function obj = SearchPathToGoal(obj,r_tree,map,pos,planning_target_k,succGraph)
              
              % retrieve goal destination
              goalState = planning_target_k.goalState;
              Target_Lanelet_ids=goalState.lanelet_ids; 
              
              % get current lanelet as position
               Curr_Lanelet_ids = r_tree.searchForLanelets(pos);
                if isempty(Curr_Lanelet_ids)
                    msg = ['cannot match x=' ,num2str(pos(1)),' ; y=',num2str(pos(2)),' of vehicle ',num2str(obj.ext_vehicle_id)];
                    error(msg);    
                end 
                
                % generate using  shortest path 
                numTargets = length(Target_Lanelet_ids);
                numCurrLanelets = length(Curr_Lanelet_ids);
                path = cell(numCurrLanelets,numTargets);
                minIndex = []; 
                minDistance = Inf;
                
                % improvement for future:
                % more efficient way would be adding a start/goal node
                % with weight 0 to all candidate lanelets for start/goal
                % point, so we only to need invoke shortestpath(...) once
                
                % calculate shortest path by checking all paths possible paths 
                % for combination of start/goal candidate lanelets
                for j = 1:numCurrLanelets
                    curr_id = Curr_Lanelet_ids(j);  
                    for k = 1:numTargets
                        % we used strings as Names for nodes, which are the
                        % lanelet ids
                        [path{j,k}, dist] = shortestpath(succGraph,num2str(curr_id),num2str(Target_Lanelet_ids(k)));         
                        if dist < minDistance
                           minDistance = min(minDistance,dist);
                           minIndex= [j,k];
                        end
                    end
                end
                
                % extract path
                obj.current_lanelet_id = Curr_Lanelet_ids(minIndex(1));
                path_ids = [];
                if isempty(minIndex)
                   error(' no path found between current position and goal state'); 
                else
                    pathString = path{minIndex(1),minIndex(2)};
                    if iscell(pathString)
                        numNodesPath = size(pathString,2);
                        path_ids = zeros(1,numNodesPath) ;
                        for k =1:numNodesPath
                            path_ids(k) = str2double(pathString{k});
                        end
                    else
                        error('output not a cell of strings');
                    end
                end
                
                %% compose centerlines
                numNodesPath = length(path_ids);
                
                % first entry is current lanelet
                Iterator = map.Search(path_ids(1)).value;           
                obj.projected_pose = Iterator.projectionOnCenterline(pos);
                [next_edge, ~] = Iterator.find_next_edge(pos); 
                obj.next_edge_index = next_edge;
                centerline = Iterator.get_centerline();
                speedProfile = Iterator.get_speedProfile();
                centerline_comp = centerline(:,next_edge:end);
                speedProfile_comp = speedProfile(1,next_edge:end);
                    
                % fully contribute to centerline: 2,...,end-1
                for k = 2:numNodesPath-1
                    obj.next_lanelet_ids(k-1) = path_ids(k);
                    Iterator = map.Search(path_ids(k)).value;
                    centerline_comp = [centerline_comp(:,1:end-1), Iterator.get_centerline()]; % end point and start point overlap, so delete end point
                    speedProfile_comp = [speedProfile_comp(:,1:end-1), Iterator.get_speedProfile()]; % end point and start point overlap, so delete end point
                    
                end
                
                % last lanelet only until position
                obj.next_lanelet_ids(numNodesPath-1) = path_ids(numNodesPath);
                Iterator = map.Search(path_ids(numNodesPath)).value;
                goalPos = goalState.position(:,minIndex(2));
                if isnan(goalPos)
                     centerline= Iterator.get_centerline();
                    goalPos = centerline(:,end);
                end
                [next_edge, ~] = Iterator.find_next_edge(goalPos); 
                centerline = Iterator.get_centerline();
                speedProfile = Iterator.get_speedProfile();
                centerline_comp = [centerline_comp(:,1:end-1), centerline(:,1:next_edge)];
                speedProfile_comp = [speedProfile_comp(:,1:end-1),speedProfile(1,1:next_edge)];
                speedProfile_comp(end) = 0;
                 
                 
                obj.path_pos = centerline_comp;
                obj.path_speed = speedProfile_comp;
 
                
                
          
        end

        
    end
    
end
