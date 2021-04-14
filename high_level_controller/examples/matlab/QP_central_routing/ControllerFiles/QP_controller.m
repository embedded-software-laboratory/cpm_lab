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



function [U,trajectoryPrediction,controllerOutput] = QP_controller( scenario,iter,prevOutput ) 
    controllerOutput = struct;

    mpc = generate_mpc_matrices( scenario,iter ); 
    qp = convert_to_QP( scenario,iter,mpc );

    % take previous output as initialization
    du_prev = prevOutput.du;
    n_du = length(du_prev);
    du = reshape(du_prev,[n_du,1]);
    du(1:end-1,:)=du(2:end,:);
    du(end,:) = 0;

    
    controllerOutput.resultInvalid = false;

    
    disp('entering optimizer');
    % try last MPC sol.
    [du , feasible , ~, controllerOutput.optimization_log] = QP_optimizer( scenario,iter,qp, mpc, du );
   disp('finished optimization');
    
    if ~feasible
            disp(['\n\n\n']);
    end
    
    controllerOutput.du=du;
    
    controllerOutput.feasible=feasible;
    
    [trajectoryPrediction,U] = decode_deltaU(scenario,iter,mpc,du);

end