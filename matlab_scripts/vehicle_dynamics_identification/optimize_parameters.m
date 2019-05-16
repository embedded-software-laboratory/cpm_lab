function optimize_parameters

    addpath('~/casadi-linux-matlabR2014b-v3.4.5')
    import casadi.*
    sequences = load('sequences.mat');
    sequences = sequences.sequences;
    
    % use random subset for testing
    sequences = sequences(randperm(length(sequences)));
    %sequences = sequences(1:20);
    
    
    init_parameters = [  0.940000, 0.000000, 0.000000, 2.941200, -2.500000, 16.500000, 0.000000, 1.620000, 5.555600, 0.000000, 0.000000, ]';
    init_parameters = [  1.002971, -0.136922, 0.235773, 3.857093, -1.167893, 10.345707, -0.796048, 1.114689, 11.214041, 0.053628, 0.018592 ]';
    init_parameters = [  0.993031, -0.125480, 0.254318, 3.420219, -1.809237, 5.068506, 0.301489, 1.258333, 8.322000, 0.001451, 0.031261 ]';
    init_parameters = [  0.975630, -0.025596, 0.254035, 3.221276, -1.644736, 7.105890, 0.118146, 1.442459, 9.033599, 0.018313, 0.022002 ]';
    init_parameters = [  0.976820, -0.031865, 0.257054, 3.239819, -1.690282, 7.508750, 0.098089, 1.456106, 9.947812, 0.011501, 0.027306 ]';
    init_parameters = [ 0.967812, 0.060636, 0.394950, 3.373651, -1.522835, 8.858272, -0.154539, 1.483766, 6.929098, 0.012479, 0.051342 ]';
    init_parameters = [ 0.989017, -0.096090, 0.244079, 3.482601, -1.589433, 1.337974, 0.759202, 1.343048, 8.717094, 0.021072, 0.023359 ]';
    init_parameters = [ 0.990848, -0.112469, 0.206194, 3.469654, -1.623418, 0.851647, 0.829819, 1.329458, 9.466841, 0.021178, 0.018267 ]';
    
    

    
    %% TODO: discrete signal delays
    
    n_sequences = length(sequences);
    n_sequence_length = length(sequences(1).x);
    
    %% Casadi Variables
    P = SX.sym('p',length(init_parameters), 1);
    X = {};
    for i_sequence = 1:n_sequences
        X{i_sequence} = [ ...
            SX.sym(['x_' num2str(i_sequence)], n_sequence_length, 1) ...
            SX.sym(['y_' num2str(i_sequence)], n_sequence_length, 1) ...
            SX.sym(['yaw_' num2str(i_sequence)], n_sequence_length, 1) ...
            SX.sym(['v_' num2str(i_sequence)], n_sequence_length, 1) ...
            SX.sym(['del_' num2str(i_sequence)], n_sequence_length, 1) ...
        ];
    end
    
    %% Conversion to flat vector
    tmp = [X{:}];
    variables_flat = [P(:); tmp(:)];
    pack_fn = casadi.Function('pack_fn', [{P} X], {variables_flat});    
    unpack_fn = casadi.Function('unpack_fn', {variables_flat}, [{P} X]);

    %% Differential equations with explicit Euler
    dt = 1/50;
    defects = [];
    for i_sequence = 1:n_sequences
        
        u = [ ...
            sequences(i_sequence).motor_command ...
            sequences(i_sequence).steering_command ...
            sequences(i_sequence).battery_voltage ...
        ];
    
        x = X{i_sequence};
        x1 = x(2:end,:);
        x0 = x(1:end-1,:);
        u0 = u(1:end-1,:);
        
        defect = x1 - x0 - dt * vehicle_dynamics(x0,u0,P);
        defects = [defects; defect(:)];
    end
    
    
    %% Optimization goal
    objective = 0;
    for i_sequence = 1:n_sequences
        error_x = X{i_sequence}(:,1) - sequences(i_sequence).x;
        error_y = X{i_sequence}(:,2) - sequences(i_sequence).y;
        error_yaw = sin(0.5*(X{i_sequence}(:,3) - sequences(i_sequence).yaw)); % use sin(x/2) to make error periodic with 2pi
        error_speed = X{i_sequence}(:,4) - sequences(i_sequence).speed;
        
        objective = objective ...
            + 10*sumsqr(error_x) ...
            + 10*sumsqr(error_y) ...
            + sumsqr(error_yaw) ...
            + sumsqr(error_speed);
    end
    
    
    %% Simulate for a feasible initialization
    X_init = {};
    for i_sequence = 1:n_sequences
        
        X_sim = nan(size(X{i_sequence}));
        
        X_sim(1,:) = [...
            sequences(i_sequence).x(1) ...
            sequences(i_sequence).y(1) ...
            sequences(i_sequence).yaw(1) ...
            sequences(i_sequence).speed(1) ...
            sequences(i_sequence).steering_command(1) ...
        ];
    
        U_sim = [ ...
            sequences(i_sequence).motor_command ...
            sequences(i_sequence).steering_command ...
            sequences(i_sequence).battery_voltage ...
        ];
    
        for k = 2:size(X_sim,1)
            X_sim(k,:) = X_sim(k-1,:) + dt * vehicle_dynamics(X_sim(k-1,:), U_sim(k-1,:), init_parameters);
        end
        X_init{i_sequence} = X_sim;
    end
    
    
    %% Initialization
    init_flat = pack_fn(init_parameters, X_init{:});
    init_flat_first = init_flat;
    
    %% Feasibility check
    defects_fn = casadi.Function('defects_fn', {variables_flat}, {defects});
    assert(all(abs(full(defects_fn(init_flat_first))) < 1e-6));

    
    %% Optimization
    n_iter = 5;
    for i = 1:n_iter
        if i < n_iter
            objective_trust_region = objective + (0.25^i) * sumsqr(init_flat-variables_flat);
        else
            objective_trust_region = objective;
        end

        nlp = struct('x', variables_flat, 'f', objective_trust_region, 'g', defects);
        nlpsolver = nlpsol('nlpsolver', 'ipopt', nlp);
        nlp_result = nlpsolver(...
            'x0', init_flat, ...
            'lbg', zeros(size(defects)), ...
            'ubg', zeros(size(defects)) ...
        );
        init_flat = nlp_result.x;
    end
    
    result_primal = cell(1,length(sequences)+1);
    [result_primal{:}] = unpack_fn(nlp_result.x);
    parameters_soln = full(result_primal{1});
    X_soln = result_primal(2:end);
    X_soln = cellfun(@(e) full(e), X_soln,  'UniformOutput',false);
    X_soln = cat(3,X_soln{:});
    
    fprintf('New Parameters:\n')
    fprintf('%f, ', parameters_soln)
    fprintf('\n')
    
    clf
    hold on
    plot(full(init_flat_first))
    plot(full(nlp_result.x))
    
    save('dump','parameters_soln','X_soln','sequences');

    
end

