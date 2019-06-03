function optimize_parameters_delay_grid
    for i = 0:2
        for j = 0:2
            for k = 0:2
                optimize_parameters(i,j,k)
            end
        end
    end
end