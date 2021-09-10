
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