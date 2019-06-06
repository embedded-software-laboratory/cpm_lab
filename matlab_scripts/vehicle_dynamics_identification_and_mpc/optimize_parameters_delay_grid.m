function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:3
        for n_delay_steps_local = 0:3
            for n_delay_steps_steering = 0:8
                for n_delay_steps_motor = 0:8
                    tic
                        optimize_parameters(n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor)
                    toc
                end
            end
        end
    end
end