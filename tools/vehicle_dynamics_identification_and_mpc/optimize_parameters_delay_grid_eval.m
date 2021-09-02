function optimize_parameters_delay_grid_eval

    files = dir('output/optimal_parameters*.mat');
    for i = 1:length(files)
        results(i) = load([files(i).folder '/' files(i).name]);
    end


    
    n_delay_steps_IPS = [results.n_delay_steps_IPS];
    n_delay_steps_local = [results.n_delay_steps_local];
    n_delay_steps_steering = [results.n_delay_steps_steering];
    n_delay_steps_motor = [results.n_delay_steps_motor];
    objective_soln = [results.objective_soln];
    
    dim_delay_steps_IPS = numel(unique(n_delay_steps_IPS));
    dim_delay_steps_local = numel(unique(n_delay_steps_local));
    dim_delay_steps_steering = numel(unique(n_delay_steps_steering));
    dim_delay_steps_motor = numel(unique(n_delay_steps_motor));
    
    N = dim_delay_steps_IPS * dim_delay_steps_local * dim_delay_steps_steering * dim_delay_steps_motor;
    assert(N == numel(n_delay_steps_IPS));
    assert(N == numel(n_delay_steps_local));
    assert(N == numel(n_delay_steps_steering));
    assert(N == numel(n_delay_steps_motor));
    
    
    [obj_min, I]=min(objective_soln);
    
    n_delay_steps_IPS_opt = n_delay_steps_IPS(I);
    n_delay_steps_local_opt = n_delay_steps_local(I);
    n_delay_steps_steering_opt = n_delay_steps_steering(I);
    n_delay_steps_motor_opt = n_delay_steps_motor(I);
    
    objective_soln = reshape(objective_soln, dim_delay_steps_motor, dim_delay_steps_steering, dim_delay_steps_local, dim_delay_steps_IPS);
    
    
    objective_soln_motor = permute(objective_soln, [1 2 3 4]);
    objective_soln_motor = reshape(objective_soln_motor, dim_delay_steps_motor, N/dim_delay_steps_motor);
    
    objective_soln_steering = permute(objective_soln, [2 1 3 4]);
    objective_soln_steering = reshape(objective_soln_steering, dim_delay_steps_steering, N/dim_delay_steps_steering);
    
    objective_soln_local = permute(objective_soln, [3 2 1 4]);
    objective_soln_local = reshape(objective_soln_local, dim_delay_steps_local, N/dim_delay_steps_local);
    
    objective_soln_IPS = permute(objective_soln, [4 2 3 1]);
    objective_soln_IPS = reshape(objective_soln_IPS, dim_delay_steps_IPS, N/dim_delay_steps_IPS);
    
    
    
    plot(objective_soln_motor,'o-')
    plot(objective_soln_steering,'o-')
    plot(objective_soln_local,'o-')
    plot(objective_soln_IPS,'o-')
    
    
    cutoff_objective = quantile(objective_soln(:),6/N);
    filter = objective_soln(:) <= cutoff_objective;
    
    relative_delays_IPS = n_delay_steps_IPS(filter) - n_delay_steps_local(filter);
    relative_delays_local = n_delay_steps_local(filter) - n_delay_steps_local(filter);
    relative_delays_steering = n_delay_steps_steering(filter) - n_delay_steps_local(filter);
    relative_delays_motor = n_delay_steps_motor(filter) - n_delay_steps_local(filter);
    
    %% Results
    % IPS delay: 1 sample
    % local delay: 1 sample
    % steering delay: 8 samples
    % motor delay: 1 sample
    
end






