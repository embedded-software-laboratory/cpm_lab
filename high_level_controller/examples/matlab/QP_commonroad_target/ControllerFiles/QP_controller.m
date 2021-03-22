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

function [U,trajectoryPrediction,controllerOutput] = QP_controller( scenario,iter,prevOutput ) 
    controllerOutput = struct;

    mpc = generate_mpc_matrices( scenario,iter ); 
    qp_problem = convert_to_QP( scenario,iter,mpc );

    %du = zeros(scenario.model.nu*scenario.Hu,1);

    % take previous output as initialization
    du_prev = prevOutput.du;
    n_du = length(du_prev);
    du = reshape(du_prev,[n_du,1]);
    du(1:end-1,:)=du(2:end,:);
    du(end,:) = 0;

    
    controllerOutput.resultInvalid = false;
    optimizerTimer = tic;
    
    disp('entering optimizer');
    % try last MPC sol.
    [du , feasible , ~, controllerOutput.optimization_log] = QP_optimizer( scenario,iter,qp_problem, mpc, du );
   disp('finished optimization');
    
    if ~feasible.feasibleColl
            disp(['infeasible point due collision with du =']);
            disp([num2str(du)]);
            disp('collision constraint eval:');
            collision = qp_problem.A_coll * du - qp_problem.b_coll;
            for k = 1: length(collision)
                disp(num2str(collision(k)));
            end     
    end
    
    controllerOutput.du=du;
    
    controllerOutput.feasible=feasible;
    
    [trajectoryPrediction,U] = decode_deltaU(scenario,iter,mpc,du);
    controllerOutput.optimizerTime = toc(optimizerTimer);
end