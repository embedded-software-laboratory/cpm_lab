function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:3
        for n_delay_steps_local = 0:3
            for n_delay_steps_command = 0:5
                tic
                optimize_parameters(n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_command)
                toc
            end
        end
    end
end