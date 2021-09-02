function [trajPredPos,trajPredSpeed, du, feasible] = Priority_QP(curr_state,mpc_info)

    scenario.currPos = curr_state.pos; % current position is always zero because we have transformed the reference traj
    currSpeed = max(0,curr_state.speed); % set to positive speed if vehicle is moving backwards

    currAcc = 0;
    

    % A tick is the shortest timespan considered during simulation.
    % All other time constants must be integral multiples. [s]
    scenario.dt   = mpc_info.dt; % MPC sample time [s]
    scenario.Hp   = mpc_info.Hp; % Prediction horizon
    scenario.Hu   = mpc_info.Hu; % Control horizon
    scenario.dynamicObstacles = mpc_info.dynamicObstacles;
    scenario.nObst = mpc_info.nObst;
    scenario.H_coll = scenario.Hp-1;   
    
    AccelerationLimit = 2;  % [m/s^2] Beschleunigung
    du_lim = 0.5 ; % [m/s^2] Beschleunigung
    scenario.du_lim_max = du_lim;
    scenario.du_lim_min = -du_lim;
    scenario.u_lim_max = AccelerationLimit ;
    scenario.u_lim_min = -AccelerationLimit;
    
    scenario.v_lim_min = mpc_info.minSpeed;; % vehicle model causes problem at low speed
    %scenario.v_lim_max = 999; % unlimited
    
    scenario.model = mpc_info.model;
    
    
    
    
    % Trajectory deviation weights for MPC, assumes diagonal matrix, 
    % else change mpc_cost_function_matrices.m
    scenario.Q = [1 100]; % Q is produced by diag(...)
    scenario.Q_final = [1 1]; % Trajectory deviation weight for the final prediction step    
    scenario.R = [0.1]; % Input weights for MPC, acc
    

    % safety distance collision
    scenario.dsafeVehicles = mpc_info.dsafeVehicles; % [m]

    
    % Vehicle state: column=[progress; speed; Acc]';
    % units:     [m; m/s;  m/s^2 ]
    x0 = [0;currSpeed;currAcc;]; 
    scenario.x0 = x0;
    
    % column = [initial_acceleration] ; [radians; m/s^2  ]
    scenario.u0 = [currAcc]; 
    
    % The reference trajectory for the vehicle and the desired speed. [m]
    scenario.path_pos  = mpc_info.path_pos;
    scenario.path_speed  = mpc_info.path_speed;
 
    u_prev = scenario.u0;
    prevOutputs.du = mpc_info.du_prev ; % Output of previous iteration, unshifted.

    % number of vehicles with higher priority
    scenario.nObst = mpc_info.nObst;
    
	%%

    
    [iter] = MPC_init(scenario, x0, u_prev);
    [U,local_trajectoryPredictions,controllerOutputs] = QP_controller( scenario,iter,prevOutputs );

   
    trajPredPos = local2global(local_trajectoryPredictions(1,:),mpc_info.path_pos,curr_state.pos);
    trajPredSpeed = local_trajectoryPredictions(2,:);
%     prevOutputs.du = controllerOutputs.du';
    du = controllerOutputs.du;
    feasible = controllerOutputs.feasible;
    
    
