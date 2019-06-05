function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:4
        for n_delay_steps_local = 0:4
            for n_delay_steps_command = 0:9
                tic
                optimize_parameters(n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_command)
                toc
            end
        end
    end
end