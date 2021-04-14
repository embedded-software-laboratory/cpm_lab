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