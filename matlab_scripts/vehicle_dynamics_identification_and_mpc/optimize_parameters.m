function optimize_parameters(file_name, n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor)

    %% See 'optimize_parameters.png/tex' for documentation.

    addpath('~/casadi-linux-matlabR2014b-v3.4.5')
    import casadi.*
    sequences = load('output/sequences.mat');
    sequences = sequences.sequences;
    
    % use random subset for testing
    sequences = sequences(randperm(length(sequences)));
    %sequences = sequences(1:20);
    
    % optimal delay (1,1,8,0), chosen delay (1,1,8,1)
    init_parameters = [1.00200223280075;-0.120981375817052;0.210836071655533;3.55505337083871;-1.4236956738529;6.90424987304674;1.33501029339947;0.0319884021481027;-6.29318567089044];
    
    %% Emulate delays by shifting data
    n_delay_max = 10;
    
    assert(n_delay_steps_IPS < n_delay_max);
    assert(n_delay_steps_local < n_delay_max);
    assert(n_delay_steps_steering < n_delay_max);
    assert(n_delay_steps_motor < n_delay_max);
    assert(n_delay_max < length(sequences(1).x));
    
    for i = 1:length(sequences)
        sequences(i).x                = circshift(sequences(i).x,                 n_delay_steps_IPS,     1);
        sequences(i).y                = circshift(sequences(i).y,                 n_delay_steps_IPS,     1);
        sequences(i).yaw              = circshift(sequences(i).yaw,               n_delay_steps_IPS,     1);
        sequences(i).speed            = circshift(sequences(i).speed,             n_delay_steps_local,   1);
        sequences(i).steering_command = circshift(sequences(i).steering_command,  n_delay_steps_steering,1);
        sequences(i).motor_command    = circshift(sequences(i).motor_command,     n_delay_steps_motor,   1);
        % sequences(i).battery_voltage  = circshift(sequences(i).battery_voltage,   n_delay_steps_local,   1);
    end
    
    % Trim start data, that wrapped around in circshift()
    f_names = fieldnames(sequences);
    for i = 1:length(sequences)
        for j = 1:length(f_names)
            f_name = f_names{j};
            data = sequences(i).(f_name);
            sequences(i).(f_name) = data((n_delay_max+1):end);
        end
    end
    
    
    
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
            %SX.sym(['del_' num2str(i_sequence)], n_sequence_length, 1) ...
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
            % sequences(i_sequence).battery_voltage ...
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
            %sequences(i_sequence).steering_command(1) ...
        ];
    
        U_sim = [ ...
            sequences(i_sequence).motor_command ...
            sequences(i_sequence).steering_command ...
            % sequences(i_sequence).battery_voltage ...
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
    regularization_weights = [1e-2 1e-9];
    for i = 1:length(regularization_weights)
        objective_trust_region = objective + regularization_weights(i) * sumsqr(init_flat-variables_flat);
        
        nlp = struct('x', variables_flat, 'f', objective_trust_region, 'g', defects);
        nlpsolver = nlpsol('nlpsolver', 'ipopt', nlp);
        nlp_result = nlpsolver(...
            'x0', init_flat, ...
            'lbg', zeros(size(defects)), ...
            'ubg', zeros(size(defects)) ...
        );
    
        % Check objectives
        tmp_fn = casadi.Function('tmp_fn', {variables_flat}, {objective, regularization_weights(i) * sumsqr(init_flat-variables_flat)});
        [original_objective, regularization_objective] = tmp_fn(nlp_result.x)
    
    
        init_flat = nlp_result.x;
    end
    
    result_primal = cell(1,length(sequences)+1);
    [result_primal{:}] = unpack_fn(nlp_result.x);
    parameters_soln = full(result_primal{1});
    X_soln = result_primal(2:end);
    X_soln = cellfun(@(e) full(e), X_soln,  'UniformOutput',false);
    X_soln = cat(3,X_soln{:});
    
    objective_soln = full(nlp_result.f);
    
    save(file_name,...
        'parameters_soln','X_soln','sequences', 'objective_soln', 'n_delay_steps_IPS', 'n_delay_steps_local', 'n_delay_steps_steering', 'n_delay_steps_motor');
    
    fprintf('New Parameters:\n')
    fprintf('%f, ', parameters_soln)
    fprintf('\n')
    
    
%     
%     figure(1)
%     clf
%     hold on
%     plot(full(init_flat_first))
%     plot(full(nlp_result.x))
%     
%     
%     
%     
%     dx = ([sequences.x] - squeeze(X_soln(:,1,:)));
%     dy = ([sequences.y] - squeeze(X_soln(:,2,:)));    
%     dist = sqrt(dx.^2 + dy.^2);
%     figure(2)
%     clf
%     histogram(dist(:))
    
end

