
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
