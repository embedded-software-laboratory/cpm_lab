classdef PlanningProblem < handle
    
   properties
       id;
       initialState;
       numGoalState = int8(0);
       goalState;
       
       
   end
   methods
       
       function obj = PlanningProblem(New_PlanningProblem)
           numAttributes = size(New_PlanningProblem.Attributes,1);
           for k=1:numAttributes
               if strcmp(New_PlanningProblem.Attributes.Name, 'id')
                   obj.id = New_PlanningProblem.Attributes.Value;
               end
           end
           numChildren = size(New_PlanningProblem.Children,2);
           for k=1:numChildren
               switch New_PlanningProblem.Children(k).Name
                   
                   case 'initialState'
                       obj.initialState = stateTostruct(New_PlanningProblem.Children(k));
                       
                   case 'goalState'
                       obj.numGoalState = obj.numGoalState+1;
                       obj.goalState{1,obj.numGoalState} = goalstateTostruct(New_PlanningProblem.Children(k));
                       
               end
               
           end
       end
   end
       

    
    
    
    
end