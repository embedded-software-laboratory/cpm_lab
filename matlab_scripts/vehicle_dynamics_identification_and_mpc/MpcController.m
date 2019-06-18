classdef MpcController
    %MPCCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        u_soln
        parameters
        Hp
        Hu
        mpc_fn
        dt
        momentum
        mpc_fn_compiled
    end
    
    methods
        function obj = MpcController(parameters, Hp, Hu, dt)
            obj.parameters = parameters;
            obj.Hp = Hp;
            obj.Hu = Hu;
            obj.dt = dt;
            u_idx = ceil((1:Hp)/Hp*Hu);
            

            
            % setenv('PATH', [getenv('PATH') ':/home/janis/casadi-linux-matlabR2014b-v3.4.5'])
            addpath('~/casadi-linux-matlabR2014b-v3.4.5')
            import casadi.*
            
            var_x0 = SX.sym('x', 1, 4);
            var_u = SX.sym('ui', Hu, 3);
            var_momentum = SX.sym('M', Hu, 3);
            var_params = SX.sym('p', length(parameters), 1);
            var_reference_trajectory_x = SX.sym('rx', Hp, 1);
            var_reference_trajectory_y = SX.sym('ry', Hp, 1);
            var_learning_rate = SX.sym('LR', 1, 1);
            var_momentum_rate = SX.sym('MR', 1, 1);
            
            X = var_x0 + dt * vehicle_dynamics(var_x0, var_u(u_idx(1),:), var_params);
            for k = 2:Hp
                X = [X;...
                    X(end,:) + dt * vehicle_dynamics(X(end,:), var_u(u_idx(k),:), var_params) ...
                ];
            end
            
            trajectory_x = X(:,1);
            trajectory_y = X(:,2);
            
            weights = ones(size(trajectory_x));
%             weights(end) = 2;
            
            objective = sumsqr(weights .* (trajectory_x - var_reference_trajectory_x)) ...
                      + sumsqr(weights .* (trajectory_y - var_reference_trajectory_y));
            
            opt_vars = var_u(:,1:2);            
            opt_vars = reshape(opt_vars, Hu*2, 1);
            
            g = gradient(objective, opt_vars);
            step_u = [reshape(-g,Hu,2) zeros(Hu,1)];
            
            
            
            var_momentum_next = var_momentum_rate * var_momentum + step_u;
            var_u_next = var_u + var_learning_rate * var_momentum_next;
            
            
            obj.mpc_fn = casadi.Function('casadi_mpc_fn', ...
                {var_x0, var_u, var_momentum, var_params, var_reference_trajectory_x, var_reference_trajectory_y, var_learning_rate, var_momentum_rate},...
                {trajectory_x, trajectory_y, objective, var_momentum_next, var_u_next},...
                {'var_x0', 'var_u', 'var_momentum', 'var_params', 'var_reference_trajectory_x', 'var_reference_trajectory_y', 'var_learning_rate', 'var_momentum_rate'}, ...
                {'trajectory_x', 'trajectory_y', 'objective', 'var_momentum_next', 'var_u_next'} );
            
            obj.mpc_fn.generate('casadi_mpc_fn.c',struct('with_header',true));
            
            C = Importer('casadi_mpc_fn.c','clang');
            obj.mpc_fn_compiled = external('casadi_mpc_fn',C);
            
            
            obj.u_soln = [0.01,0.01,8] .* ones(Hu,3);
            
            obj.momentum = 0*obj.u_soln;
            
            
            copyfile casadi_mpc_fn.c ../../vehicle_raspberry_firmware/src/
            copyfile casadi_mpc_fn.h ../../vehicle_raspberry_firmware/src/
        end
        
        function [u, trajectory_pred_x, trajectory_pred_y] = update(obj, state, reference_trajectory_x, reference_trajectory_y)
            
            
            learning_rate = 0.5;
            momentum_rate = 0.6;
            
            objective_prev = 1e111;
                
            tic
            for j = 1:100
                [trajectory_x, trajectory_y, objective, momentum_next, u_next] = ...
                    obj.mpc_fn_compiled(state, obj.u_soln, obj.momentum, obj.parameters, ...
                    reference_trajectory_x, reference_trajectory_y, learning_rate, momentum_rate);

                
                obj.momentum = full(momentum_next);
                
                obj.u_soln = full(u_next);
                
                obj.u_soln(:,1:2) = min(1,max(-1,obj.u_soln(:,1:2)));
                obj.u_soln(:,3) = 8;
                
                objective_change = objective_prev - objective;
                
%                 if( j > 10 && full(objective_change) < 1e-5 )
%                     j
%                     break;
%                 end
                
                objective_prev = objective;
            end
            toc
            fprintf('%.7f\n',full(objective));
            
            trajectory_pred_x = full(trajectory_x);
            trajectory_pred_y = full(trajectory_y);
            
            u = obj.u_soln(1,:);
        
        end
    end
end

