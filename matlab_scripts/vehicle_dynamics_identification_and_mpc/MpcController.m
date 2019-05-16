classdef MpcController
    %MPCCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        u_soln
        parameters
        Hp
        Hu
        mpc_fn
    end
    
    methods
        function obj = MpcController(parameters, Hp, Hu)
            obj.parameters = parameters;
            obj.Hp = Hp;
            obj.Hu = Hu;
            u_idx = ceil((1:Hp)/Hp*Hu);
            

            addpath('~/casadi-linux-matlabR2014b-v3.4.5')
            import casadi.*
            
            dt = 1/50;
            
            var_x0 = SX.sym('x', 1, 5);
            var_u = SX.sym('ui', Hu, 3);
            var_params = SX.sym('p', length(parameters), 1);
            var_reference_trajectory_x = SX.sym('rx', Hp, 1);
            var_reference_trajectory_y = SX.sym('ry', Hp, 1);
            
            X = var_x0 + dt * vehicle_dynamics(var_x0, var_u(u_idx(1),:), var_params);
            for k = 2:Hp
                X = [X;...
                    X(end,:) + dt * vehicle_dynamics(X(end,:), var_u(u_idx(k),:), var_params) ...
                ];
            end
            
            trajectory_x = X(:,1);
            trajectory_y = X(:,2);
            
            objective = sumsqr(trajectory_x - var_reference_trajectory_x) + sumsqr(trajectory_y - var_reference_trajectory_y);
            
            opt_vars = var_u(:,1:2);            
            opt_vars = reshape(opt_vars, Hu*2, 1);
            
            g = gradient(objective, opt_vars);
            %H = hessian(objective, opt_vars);
            %step = -H\g;
            %step_u = [reshape(step,Hu,2) zeros(Hu,1)];
            step_u = [reshape(-g,Hu,2) zeros(Hu,1)];
            
            
            obj.mpc_fn = casadi.Function('mpc_fn', ...
                {var_x0, var_u, var_params, var_reference_trajectory_x, var_reference_trajectory_y},...
                {trajectory_x, trajectory_y, objective, step_u});
            
            
            obj.u_soln = [0.01,0.01,8] .* ones(Hu,3);
        end
        
        function [u, trajectory_pred_x, trajectory_pred_y] = update(obj, state, reference_trajectory_x, reference_trajectory_y)
            
            for j = 1:150
                [trajectory_x, trajectory_y, objective, step_u] = ...
                    obj.mpc_fn(state, obj.u_soln, obj.parameters, reference_trajectory_x, reference_trajectory_y);

                obj.u_soln = obj.u_soln + 0.03*full(step_u);
                %obj.u_soln = [1,0,8] .* [1;1];
            end
            fprintf('%.7f\n',full(objective));
            
            trajectory_pred_x = full(trajectory_x);
            trajectory_pred_y = full(trajectory_y);
            
            u = obj.u_soln(1,:);
        
        end
    end
end

