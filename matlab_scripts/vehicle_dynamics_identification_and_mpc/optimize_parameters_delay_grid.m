function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:1
        for n_delay_steps_local = 0:1
            for n_delay_steps_steering = 2:5
                for n_delay_steps_motor = 2:5
                    tic
                        optimize_parameters(n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor)
                    toc
                end
            end
        end
    end
end