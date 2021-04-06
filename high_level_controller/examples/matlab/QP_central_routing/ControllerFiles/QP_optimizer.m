% Copyright 2016 Bassam Alrifaee
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as 
% published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

function [ x_0 , feasible , objValue, optimization_log] = QP_optimizer( scenario,iter,qcqp, mpc, x_0 )

if abs(x_0(1)) < eps % avoid numerical issues
    x_0(1) = eps;
end


nObst = scenario.nObst;
Hp = scenario.Hp;
Hu = scenario.Hu;
nu = scenario.model.nu;

n_slack = length(qcqp.b_coll); % as many slack as collision steps
n_du = nu*Hu;
idx_slack = n_du+1 : n_du+n_slack;
idx_du = 1:n_du;

nVars = n_du+n_slack;
nCons = qcqp.nCons;

% n_stateCons = size(qcqp.A,1);


[ ~, objValue_0,  max_violation_0, ~ ] = QP_evaluate( scenario, qcqp, x_0);
cfg = config;
delta_tol = 1e-4;
slack_weight = 1e10;
slack_ub = 1e20;
slack_lb = 0;


optimization_log = struct.empty;

for i = 1:1
    Aineq = zeros(nCons,nVars);
    bineq = zeros(nCons,1);
    row = 0;
    lb= zeros(nVars,1);
    ub= zeros(nVars,1);
    
    

    
    % U limits and minimum speed
    row = row(end) + (1:length(qcqp.b));
    Aineq(row,idx_du) = qcqp.A;
    bineq(row) = qcqp.b;
   
    
    % COLLISION AVOIDANCE
    row = row(end) + (1:length(qcqp.b_coll));
    Aineq(row,idx_du) = qcqp.A_coll;
    bineq(row) = qcqp.b_coll;   
    Aineq(row,idx_slack)  = -1*eye(n_slack);
    
    
    
    % BOX CONSTRAINTS
    lb(idx_du) = qcqp.lb_du_min ;
    ub(idx_du) = qcqp.ub_du_max ;
    lb(idx_slack) = slack_lb * ones(n_slack,1);
    ub(idx_slack) = slack_ub * ones(n_slack,1);
    
    % slack variable
    q = [qcqp.q0; slack_weight* ones(n_slack,1)];
    P = qcqp.p0;
    
    % weight of slack for Hessian. 
    quad_slack_weight = qcqp.p0(1,1); % 
    
    P(idx_slack,idx_slack) = quad_slack_weight*ones(n_slack,n_slack);
    
    
    [x, fval, exitflag, output] = cplexqp(P, q, Aineq, bineq, [], [], lb, ub);

    %     exitflag = -2 means infeasible
    if exitflag == 1 || exitflag == 5 || exitflag == 6
        
        slack = x(idx_slack);
        prev_x = x_0;
        x_0 = x(idx_du);
        % Eval. progress
        [ feasible, objValue,  max_violation, sum_violations  ] = QP_evaluate( scenario, qcqp, x_0);
       

        max_constraint =max_violation;
        
        fval = fval + qcqp.r0;
        delta_hat = (objValue_0 + slack_weight*max_violation_0) - fval; % predicted decrease of obj
        delta = (objValue_0 + slack_weight*max_violation_0) - (objValue + slack_weight*max_violation); % real decrease of obj
        fprintf('slack %8f max_violation %8f sum_violations %8f feas %d objVal %8f fval %8f max_constraint %e\n',max(slack), max_violation, sum_violations, feasible, objValue, fval, max_constraint);
        
        objValue_0 = objValue;
        max_violation_0 = max_violation;
                
        % Log context
        optimization_log(i).P = P;
        optimization_log(i).q = q;
        optimization_log(i).Aineq = Aineq;
        optimization_log(i).bineq = bineq;
        optimization_log(i).lb = lb;
        optimization_log(i).ub = ub;
        optimization_log(i).x = x;
        optimization_log(i).slack = slack;
        optimization_log(i).SCP_ObjVal = fval;
        optimization_log(i).QCQP_ObjVal = objValue;
        optimization_log(i).delta_hat = delta_hat;
        optimization_log(i).delta = delta;
        optimization_log(i).du = x_0;
        optimization_log(i).feasible = feasible;
        optimization_log(i).prev_du = prev_x;
        [optimization_log(i).Traj,optimization_log(i).U] = decode_deltaU(scenario,iter,mpc,x_0);
        [optimization_log(i).prevTraj,optimization_log(i).prevU] = decode_deltaU(scenario,iter,mpc,prev_x);
        
        
 
        if  abs(delta) < delta_tol && max_violation <= cfg.QCQP.constraintTolerance % max_violation is constraintTolerance in QCQP_evaluate.m.
            break
        end
    else
        disp(['exitflag: ' num2str(exitflag) ', ' output.cplexstatusstring ', ' output.message]);
        feasible = false;
        objValue = 9999999999;
        return;
    end
end
disp(['iterations: ' num2str(i)])
end

