function main

    discrete_model_fn = discretize_linearize(1/50);
    
    [Q, RL, RP] = model_noise_covariance;
    
    x = randn(7,1);
    u = randn(2,1);

    [x_next, Ad, Bd, CL, CP, zL, zP] = discrete_model_fn( x, u );
    
    P = eye(7);
    z_measured = zL + RL * randn(2,1);
    
    [x_est,P] = kalman_filter_step(x_next,zL,z_measured,P,Ad,CL,Q,RL);
        
end

