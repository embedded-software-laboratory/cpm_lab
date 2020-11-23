function [planning_target] = planning_extraction(Planning_Data,r_tree, map)


% planning_target{k}.goalState.lanelet_ids 
% planning_target{k}.goalState.proj_position 
% planning_target{k.initialState.lanelet_ids 
% planning_target{k}.initialState.position 

numPlans = Planning_Data.numPlanning ;
planning_target=cell(1,numPlans);

% initialStates
for plan = 1:numPlans
     
     position = Planning_Data.Plans{1,plan}.initialState.position;
     lanelet_ids = r_tree.searchForLanelets(position);
      if ~isnan(lanelet_ids)
          planning_target{1,plan}.initialState.lanelet_ids = lanelet_ids;
            planning_target{1,plan}.initialState.position = position;
      else
          error('lanelet_id not found');
      end
end
lanelet_ids_all = [];
position_all = [];
% goalStates
for plan = 1:numPlans
  numGoals = size(  Planning_Data.Plans{1,plan}.goalState,2);
  for goal= 1 : numGoals
      position = Planning_Data.Plans{1,plan}.goalState{1,goal}.position;
      lanelet_ids = NaN;
      switch position.type
          case 'lanelet'
              lanelet_ids = position.values;
              position_xy  = [NaN ; NaN];
          case 'rectangle'
              % center of rectangle as target
              if length(position.values) == 5
                  center_xy = position.values(4:5);
              else
                  center_xy = [0,0];
              end
              
              lanelet_ids = r_tree.searchForLanelets(center_xy);
              position_xy  = center_xy;
              if size(position_xy,1) == 1
                  position_xy = position_xy';
              end
          case 'circle'
              center_xy = position.values(2:3);
              lanelet_ids = r_tree.searchForLanelets(center_xy);
              position_xy  = center_xy;
              if size(position_xy,1) == 1
                  position_xy = position_xy';
              end
          case 'polygon'
              error('polygon shape not supported');
          otherwise
              error(['unknown case : ', position.type]);
              
      end
      if ~isnan(lanelet_ids)
          numCandidates =length(lanelet_ids); % number of lanelets which all cover this point
          position_xy_rep = position_xy * ones(1,numCandidates);
          lanelet_ids_all = [lanelet_ids_all,lanelet_ids];
          position_all = [position_all,position_xy_rep];
          
      else
          error('lanelet_id not found');
      end
  end
  planning_target{1,plan}.goalState.lanelet_ids = lanelet_ids_all;
  planning_target{1,plan}.goalState.position = position_all;
  % reset for next iteration of plan
  lanelet_ids_all = [];
  position_all = [];
end





end