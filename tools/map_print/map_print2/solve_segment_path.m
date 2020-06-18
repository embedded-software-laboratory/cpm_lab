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

function path = solve_segment_path(start_pose, end_pose)

    % start_pose = [x y yaw curvature]
    % end_pose = [x y yaw curvature]

    assert(size(start_pose, 1) == 1);
    assert(size(start_pose, 2) == 4);
    assert(size(end_pose, 1) == 1);
    assert(size(end_pose, 2) == 4);
    
    %curvature_max = 3.3;
    
    import casadi.*;

    
    state_names = {'px', 'py', 'yaw', 'curvature'};
    input_names = {'dk'};
    n_nodes = 20;
    n_states = length(state_names);
    n_inputs = length(input_names);
    
    [nodes, w_integration, D, w_interp] = collocation_constants('LGL', n_nodes-1, 0, 1);
    
    
    %% Create CasADi variables
    T = casadi.SX.sym('T');
    variables_flat = [T];
    
    X = [];        
    for i_state = 1:n_states
        X = [X casadi.SX.sym(state_names{i_state}, n_nodes)];
    end
    variables_flat = [variables_flat; X(:)];

    U = [];        
    for i_input = 1:n_inputs
        U = [U casadi.SX.sym(input_names{i_input}, n_nodes)];
    end
    variables_flat = [variables_flat; U(:)];
    
    pack_fn = casadi.Function('pack_fn', [{T} {X} {U}], {variables_flat});
    unpack_fn = casadi.Function('unpack_fn', {variables_flat}, [{T} {X} {U}]);
    
        
    %% ODE collocation defects
    equations = [];
    equations = [equations; (reshape(D*X - T .* my_ode(X, U), n_states * n_nodes,1))];
    
    %% Objective
    objective = 10*T;    
    objective = objective + T * (w_integration' * sum(U.^2,2));
    
    %% Box constraints
    X_ub =  inf(n_nodes, n_states);
    X_lb = -inf(n_nodes, n_states);
    U_ub =  inf(n_nodes, n_inputs);
    U_lb = -inf(n_nodes, n_inputs);
    T_ub = 1000;
    T_lb = 0.001;
    
    % Maximum curvature
%     X_ub(:,4,:) =  curvature_max;
%     X_lb(:,4,:) = -curvature_max;
    
    % Initial + final conditions
    X_ub(1,:) = start_pose;
    X_lb(1,:) = start_pose;
    X_ub(end,:) = end_pose;
    X_lb(end,:) = end_pose;    
    
    ub_flat = pack_fn(T_ub, X_ub, U_ub);
    lb_flat = pack_fn(T_lb, X_lb, U_lb);
    
    
    
    %% Initial guess    
    x_guess = start_pose + nodes * (end_pose - start_pose);
    u_guess = zeros(n_nodes, n_inputs);
    t_guess = norm(end_pose(1:2) - start_pose(1:2)) * 1.2;
    guess_flat = pack_fn(t_guess, x_guess, u_guess);
    guess_flat = guess_flat + 0.001 * randn(size(guess_flat));
    
    
    nlp = struct('x', variables_flat, 'f', objective, 'g', equations);
    nlpsolver = nlpsol('nlpsolver', 'ipopt', nlp);
    nlp_result = nlpsolver(...
        'x0', guess_flat, ...
        'lbg', zeros(size(equations)), ...
        'ubg', zeros(size(equations)), ...
        'lbx', lb_flat, ...
        'ubx', ub_flat...
    );
        
    [T_opt, X_opt, U_opt] = unpack_fn(nlp_result.x);
    
    T_opt = full(T_opt);
    X_opt = full(X_opt);
    U_opt = full(U_opt);
    
    T_interp = linspace(min(nodes), max(nodes), 5000);
    interpolation_matrix = Lagrange_interpolation_matrix(nodes, T_interp, w_interp);
    X_interp = interpolation_matrix * X_opt;
    %U_interp = interpolation_matrix * U_opt;
    
%     clf    
%     subplot(121)
%     hold on
%     axis equal
%     plot(X_interp(:,1), X_interp(:,2))
%         
%     subplot(122)
%     hold on
%     plot(T_interp*T_opt, X_interp(:,4))
    
    
    path = struct;
    path.s = (T_interp*T_opt)';
    path.x = X_interp(:,1);
    path.y = X_interp(:,2);
    path.yaw = X_interp(:,3);
    path.curvature = X_interp(:,4);  
    path.nodes = X_interp;
    
end


function dx = my_ode(x,u)
    dx = [cos(x(:,3)) sin(x(:,3)) x(:,4) u(:,1)];
end

