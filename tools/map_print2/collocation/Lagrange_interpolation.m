function x_interp = Lagrange_interpolation(ti, xi, t_interp, interpolation_weights)

    assert(size(ti, 2) == 1);
    assert(size(xi, 2) == 1);
    assert(size(interpolation_weights, 2) == 1);
    assert(size(t_interp, 1) == 1);

    x_interp = sum(interpolation_weights .* xi ./ (t_interp - ti),1) ./ sum(interpolation_weights ./ (t_interp - ti),1);   
    assert( ~any(isnan(x_interp)) );

end