function main

    clc
    discrete_model_fn = discretize_linearize(1/50);
    
    [Q, R] = model_noise_covariance;
    
    x = randn(3,1);
    u = randn(2,1);

    [x_next, Ad, Bd, C, z] = discrete_model_fn( x, u );
    
    P = eye(3);
    z_measured = z + R * randn(3,1);
    
    [x_est,P] = kalman_filter_step(x_next,z,z_measured,P,Ad,C,Q,R);
        
end

