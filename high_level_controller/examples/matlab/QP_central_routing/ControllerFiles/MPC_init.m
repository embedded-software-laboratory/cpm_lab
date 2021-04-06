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


% Preprocessing step for MPC controller.

% x_measured is column vector of current measurement. u_prev is a column vector of last input
function [iter] = MPC_init(scenario, x_measured, u_prev)

    iter = struct;
    nu=scenario.model.nu;

    % we start from previous last solution
    if size ( u_prev,2) ~= nu
        u_prev = u_prev';
    end
    iter.u0= u_prev;


    iter.x0 = x_measured;

    % generate reference trajectory for process variables.
    iter.RefData = sampleReferenceTrajectory(...
        scenario.Hp, ... % number of prediction steps
        scenario.path_pos,...
        scenario.path_speed,...
        scenario.currPos,...
        scenario.dt);  % distance traveled in one timestep


    
end
