function optimize_parameters_delay_grid_eval

    files = dir('output/optimal_parameters*.mat');
    for i = 1:length(files)
        results(i) = load([files(i).folder '/' files(i).name]);
    end


    
    n_delay_steps_IPS = [results.n_delay_steps_IPS];
    n_delay_steps_local = [results.n_delay_steps_local];
    n_delay_steps_command = [results.n_delay_steps_command];
    objective_soln = [results.objective_soln];
    
    
    [obj_min, I]=min(objective_soln);
    
    n_delay_steps_IPS_opt = n_delay_steps_IPS(I);
    n_delay_steps_local_opt = n_delay_steps_local(I);
    n_delay_steps_command_opt = n_delay_steps_command(I);
    
    plot(objective_soln,'o-')
end

