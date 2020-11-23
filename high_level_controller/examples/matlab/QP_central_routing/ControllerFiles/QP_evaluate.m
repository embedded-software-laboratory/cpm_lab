
% Check QP constraints for feasiblity
function [ ...
    feasible,...
    objValue, ...
    max_violation, ...
    sum_violations ] = QP_evaluate( scenario, qp, deltaU )

    cfg = config;
    sum_violations = 0;
    max_violation = 0;

    feasible = true;
%     Hp = scenario.Hp;
%     H_coll =scenario.H_coll;
    nObst = scenario.nObst;
    
   
    
    
    objValue = deltaU'*qp.p0*deltaU + qp.q0'*deltaU + qp.r0;
 

    % VEHICLES
    if nObst > 0

            ci = (qp.A_coll * deltaU-qp.b_coll);
            
            

            if (max(ci) > cfg.QCQP.constraintTolerance)
                sum_violations = sum((ci>0).*ci);
                max_violation = max(ci);
                feasible = false;
            end

    end
    
    
    
end