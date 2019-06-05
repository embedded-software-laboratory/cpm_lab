function optimize_parameters_delay_grid_eval

    files = dir('output/optimal_parameters*.mat');
    for i = 1:length(files)
        results(i) = load([files(i).folder '/' files(i).name]);
    end


    
    n_delay_steps_IPS = [results.n_delay_steps_IPS];
    n_delay_steps_local = [results.n_delay_steps_local];
    n_delay_steps_command = [results.n_delay_steps_command];
    objective_soln = [results.objective_soln];
    
    dim_delay_steps_IPS = numel(unique(n_delay_steps_IPS));
    dim_delay_steps_local = numel(unique(n_delay_steps_local));
    dim_delay_steps_command = numel(unique(n_delay_steps_command));
    
    N = dim_delay_steps_IPS * dim_delay_steps_local * dim_delay_steps_command;
    assert(N == numel(n_delay_steps_IPS));
    assert(N == numel(n_delay_steps_local));
    assert(N == numel(n_delay_steps_command));
    
    
    [obj_min, I]=min(objective_soln);
    
    n_delay_steps_IPS_opt = n_delay_steps_IPS(I);
    n_delay_steps_local_opt = n_delay_steps_local(I);
    n_delay_steps_command_opt = n_delay_steps_command(I);
    
    objective_soln = reshape(objective_soln, dim_delay_steps_command, dim_delay_steps_local, dim_delay_steps_IPS);
    
    
    objective_soln_cmd = reshape(objective_soln, dim_delay_steps_command, dim_delay_steps_local*dim_delay_steps_IPS);
    
    objective_soln_local = permute(objective_soln, [2 1 3]);
    objective_soln_local = reshape(objective_soln_local, dim_delay_steps_local, dim_delay_steps_command*dim_delay_steps_IPS);
    
    objective_soln_IPS = permute(objective_soln, [3 2 1]);
    objective_soln_IPS = reshape(objective_soln_IPS, dim_delay_steps_IPS, dim_delay_steps_command*dim_delay_steps_local);
    
    
    plot(objective_soln_cmd,'o-')
    plot(objective_soln_local,'o-')
    plot(objective_soln_IPS,'o-')
    
    
    cutoff_objective = quantile(objective_soln(:),12/N);
    filter = objective_soln(:) <= cutoff_objective;
    
    relative_delays_local = n_delay_steps_local(filter) - n_delay_steps_IPS(filter);
    relative_delays_command = n_delay_steps_command(filter) - n_delay_steps_IPS(filter);
    
end






