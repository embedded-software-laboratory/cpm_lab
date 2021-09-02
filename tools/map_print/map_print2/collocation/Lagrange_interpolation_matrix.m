function interpolation_matrix = Lagrange_interpolation_matrix(ti, t_interp, interpolation_weights)

    assert(size(ti, 2) == 1);
    assert(size(interpolation_weights, 2) == 1);
    assert(size(t_interp, 1) == 1);
    
    
    deltas = (t_interp - ti)';
    interpolation_matrix = nan(size(deltas));
    
    for j = 1:length(ti)
        deltas_copy = deltas;
        
        deltas_copy(:,j) = [];
        interpolation_matrix(:,j) = interpolation_weights(j) * prod(deltas_copy,2);
    end
    
end