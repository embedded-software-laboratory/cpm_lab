function optimize_parameters

    addpath('~/casadi-linux-matlabR2014b-v3.4.5')
    import casadi.*
    sequences = load('sequences.mat');
    sequences = sequences.sequences;
    
    % use random subset for testing
    sequences = sequences(randperm(length(sequences)));
    sequences = sequences(1:6);
    
    
    init_parameters = [    0.9400         0         0    2.9412   -2.5000   16.5000         0    1.6200    5.5556         0         0]';
    
    %% TODO: discrete signal delays
    
    n_sequences = length(sequences);
    n_sequence_length = length(sequences(1).x);
    n_states = 5;
    n_inputs = 3;
    
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
        
        defect = x1 - dt * vehicle_dynamics(x0,u0,P);
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
            + sumsqr(error_x) ...
            + sumsqr(error_y) ...
            + sumsqr(error_yaw) ...
            + sumsqr(error_speed);
    end
    
    
    %% Measured states for the initialization
    X_init = {};
    for i_sequence = 1:n_sequences
        X_init{i_sequence} = [ ...
            sequences(i_sequence).x ...
            sequences(i_sequence).y ...
            sequences(i_sequence).yaw ...
            sequences(i_sequence).speed ...
            zeros(size(sequences(i_sequence).x)) ...
        ];
    end
    
    
    %% Initialization
    init_flat = pack_fn(init_parameters, X_init{:});
    
    
    
    %% Optimization
    nlp = struct('x', variables_flat, 'f', objective, 'g', defects);
    nlpsolver = nlpsol('nlpsolver', 'ipopt', nlp);
    nlp_result = nlpsolver(...
        'x0', init_flat, ...
        'lbg', zeros(size(defects)), ...
        'ubg', zeros(size(defects)) ...
    );

    
    
        
    
end

