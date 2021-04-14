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


function [ x_0 , feasible , objValue, optimization_log] = QP_optimizer( scenario,iter,qcqp, mpc, x_0 )

if abs(x_0(1)) < eps % avoid numerical issues
    x_0(1) = eps;
end


nObst = scenario.nObst;
Hp = scenario.Hp;
Hu = scenario.Hu;
nu = scenario.model.nu;

n_slack = length([qcqp.b_coll;qcqp.b_goal ]); % as many slack as collision steps
n_du = nu*Hu;
idx_slack = n_du+1 : n_du+n_slack;
idx_du = 1:n_du;

nVars = n_du+n_slack;
nCons = qcqp.nCons;

% n_stateCons = size(qcqp.A,1);


[ ~, objValue_0,  max_violation_0 ] = QP_evaluate( scenario, qcqp, x_0);
cfg = config;
delta_tol = 1e-4;
slack_weight = 1e10;
slack_ub = 1e20;
slack_lb = 0;
max_SCP_iter =20;

optimization_log = struct.empty;

for i = 1:max_SCP_iter
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
    row = row(end) + (1:length([qcqp.b_coll;qcqp.b_goal ]));
    Aineq(row,idx_du) = [qcqp.A_coll;qcqp.A_goal ];
    bineq(row) = [qcqp.b_coll;qcqp.b_goal ];   
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
        [ feasible, objValue,  max_violation  ] = QP_evaluate( scenario, qcqp, x_0);
       

        max_constraint =max_violation;
        
        fval = fval + qcqp.r0;
        delta_hat = (objValue_0 + slack_weight*max_violation_0) - fval; % predicted decrease of obj
        delta = (objValue_0 + slack_weight*max_violation_0) - (objValue + slack_weight*max_violation); % real decrease of obj
        fprintf('slack %8f max_violation %8f feasColl %d objVal %8f fval %8f max_constraint %e\n',max(slack), max_violation, feasible.feasibleColl, objValue, fval, max_constraint);
        
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
        Aineq
        bineq
        x_try = 0*x;
        x_try(1) = -1;
        x_try(2) = 0.9;
        Aineq*x_try-bineq
        feasible.feasibleColl = false;
        feasible.feasibleGoal= false;
        
        return;
    end
end
disp(['iterations: ' num2str(i)])
end

