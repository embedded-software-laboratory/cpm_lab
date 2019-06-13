function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:1
        for n_delay_steps_local = 0:1
            for n_delay_steps_steering = 4:6
                for n_delay_steps_motor = 4:6
                    
                    file_name = sprintf('output/optimal_parameters_d_ips_%i_d_local_%i_d_steer_%i_d_mot_%i.mat', ...
                        n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor);
                    
                    
                    if exist(file_name, 'file')
                        fprintf('Skipping "%s"\n', file_name);
                    else
                        optimize_parameters(file_name, n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor);
                    end
                    
                end
            end
        end
    end
end