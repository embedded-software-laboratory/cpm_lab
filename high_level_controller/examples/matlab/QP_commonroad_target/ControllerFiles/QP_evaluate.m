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


% Check QP constraints for feasiblity
function [ ...
    feasible,...
    objValue, ...
    max_violation ...
     ] = QP_evaluate( scenario, qp, deltaU )

    cfg = config;
%     sum_violations = 0;
    max_violation = 0;

    feasibleColl = true;
    feasibleGoal = true;
%     Hp = scenario.Hp;
%     H_coll =scenario.H_coll;
    nObst = scenario.nObst;
    
   
    
    
    objValue = deltaU'*qp.p0*deltaU + qp.q0'*deltaU + qp.r0;
 

    % VEHICLES
    if nObst > 0

            ci = (qp.A_coll * deltaU-qp.b_coll);
            
            

            if (max(ci) > cfg.QCQP.constraintTolerance)
%                 sum_violations = sum((ci>0).*ci);
                max_violation = max(ci);
                feasibleColl = false;
            end
    end
     if ~isempty(qp.A_goal) 

            ci = (qp.A_goal * deltaU-qp.b_goal);
            
            

            if (max(ci) > cfg.QCQP.constraintTolerance)
%                 sum_violations = sum((ci>0).*ci);
                max_violation = max(ci);
                feasibleGoal = false;
            end
    end
    feasible.feasibleGoal = feasibleGoal;
    feasible.feasibleColl = feasibleColl;
    
end