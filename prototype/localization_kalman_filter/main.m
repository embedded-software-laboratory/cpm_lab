function main

    discrete_model_fn = discretize_linearize(1/50);
    
    x = randn(7,1);
    u = randn(2,1);

    [x_next, Ad, Bd, CL, CP, zL, zP] = discrete_model_fn( x, u );
        
end

