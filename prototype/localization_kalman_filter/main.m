function main

    clc
    dt = 1/50;
    
    %% Simulate Vehicle + IMU drift
    v_fn = @(t) sin(sqrt(2)*t)^2 + 1;
    yaw_fn = @(t) 0.7*t + sin(t) + 3;
    
    my_ode = @(t,x) model(x, [v_fn(t); yaw_fn(t)]);
    t_grid = 0:dt:20;
    
    [~,X] = ode45(my_ode, t_grid, [1 2 0]);
    px_true = X(:,1)';
    py_true = X(:,2)';
    yaw_true = arrayfun(yaw_fn,t_grid);
    v_true = arrayfun(v_fn,t_grid);
    
    % simulate noisy drift for the imu yaw
    yaw_offset_true = cumsum(cumsum(randn(size(t_grid))));
    yaw_offset_true = 0.7 * yaw_offset_true / max(abs(yaw_offset_true))+3;
    
    yaw_imu_true = yaw_true - yaw_offset_true;
    
    %% Simulate local measurements
    yaw_imu_measured = yaw_imu_true + 0.001 * randn(size(yaw_imu_true));
    v_measured = v_true + 0.05 * randn(size(v_true));
    
    %% Simulate IPS measurements
    px_IPS = px_true + 0.001 * randn(size(px_true));
    py_IPS = py_true + 0.001 * randn(size(py_true));
    yaw_IPS = yaw_true + 0.04 * randn(size(yaw_true));
    
    
    %% Run Kalman Filter    
    px_estimated = nan(size(px_true));
    py_estimated = nan(size(px_true));
    yaw_offset_estimated = nan(size(px_true));
    
    px_estimated(1) = 0;
    py_estimated(1) = 0;
    yaw_offset_estimated(1) = 0;

    discrete_model_fn = discretize_linearize(dt);
    [Q, R] = model_noise_covariance;
    P = eye(3);
    
    for i = 1:(length(t_grid)-1)
        
        x = [px_estimated(i); py_estimated(i); yaw_offset_estimated(i)];
        u = [v_measured(i); yaw_imu_measured(i)];
        
        [x_est, Ad, ~, C, z_est] = discrete_model_fn( x, u );
        
        z_measured = [px_IPS(i); py_IPS(i); yaw_IPS(i)];
        
        if i > 400
            [x_est,P] = kalman_filter_step(x_est,z_est,z_measured,P,Ad,C,Q,R);
        end
        
        px_estimated(i+1) = x_est(1);
        py_estimated(i+1) = x_est(2);
        yaw_offset_estimated(i+1) = x_est(3);
    end
    
    yaw_estimated = yaw_imu_measured + yaw_offset_estimated;
    
    clf
    
    subplot(221)
    hold on
    plot(t_grid, yaw_estimated,'DisplayName','est');
    plot(t_grid, yaw_IPS,'DisplayName','meas');
    plot(t_grid, yaw_true,'DisplayName','true');
    ylabel('yaw')
    legend
    
    
    
    subplot(222)
    hold on
    plot(t_grid, px_estimated,'DisplayName','est');
    plot(t_grid, px_IPS,'DisplayName','meas');
    plot(t_grid, px_true,'DisplayName','true');
    ylabel('x')
    legend
    
    
    subplot(223)
    hold on
    plot(t_grid, py_estimated,'DisplayName','est');
    plot(t_grid, py_IPS,'DisplayName','meas');
    plot(t_grid, py_true,'DisplayName','true');
    ylabel('y')
    legend
    
    subplot(224)
    hold on
    plot(t_grid, yaw_offset_estimated,'DisplayName','est');
    plot(t_grid, yaw_offset_true,'DisplayName','true');
    ylabel('yaw offset')
    legend
end

