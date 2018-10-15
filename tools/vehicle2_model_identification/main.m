function main

    clc
    vehicleState = importdata('vehicleState.csv');
    vehicleCommand = importdata('vehicleCommand.csv');    
    
    Timestamp_state = vehicleState.data(:, strcmp(vehicleState.colheaders, 'Timestamp'));
    Timestamp_command = vehicleCommand.data(:, strcmp(vehicleCommand.colheaders, 'Timestamp'));
        
    Timestamp_state = Timestamp_state - min(Timestamp_command);
    Timestamp_command = Timestamp_command - min(Timestamp_command);
    Timestamp_state = Timestamp_state / 1e9;    
    Timestamp_command = Timestamp_command / 1e9;
    
    t0 = min(min(Timestamp_state), min(Timestamp_command));
    tf = max(max(Timestamp_state), max(Timestamp_command));
    dt = median(diff(Timestamp_state));
    
    % common time grid
    T = t0:dt:tf;
    T = T(10:end-10);
    
    % interpolate to same grid
    yaw = load_timeseries(vehicleState, 'pose.yaw', Timestamp_state, T);
    x = load_timeseries(vehicleState, 'pose.x', Timestamp_state, T);
    y = load_timeseries(vehicleState, 'pose.y', Timestamp_state, T);
    odometer_distance = load_timeseries(vehicleState, 'odometer_distance', Timestamp_state, T);
    speed = load_timeseries(vehicleState, 'speed', Timestamp_state, T);
    imu_acceleration_forward = load_timeseries(vehicleState, 'imu_acceleration_forward', Timestamp_state, T);
    imu_acceleration_left = load_timeseries(vehicleState, 'imu_acceleration_left', Timestamp_state, T);
    
    
    motor_throttle = load_timeseries(vehicleCommand, 'data.direct_control.motor_throttle', Timestamp_command, T);
    steering_servo = load_timeseries(vehicleCommand, 'data.direct_control.steering_servo', Timestamp_command, T);
    
    
    
    motor_speed_fit(T, dt, speed, motor_throttle)
    
end

function motor_speed_fit(T, dt, speed, motor_throttle)

    % nonlinear parameters, optimized with trial and error
    params = [1.592861  0.031120];
    mse = 1e100;

    for i = 1:1
        params_new = params + randn(size(params)) * 0.001;
        
        % cost function
        u = motor_throttle';
        u = sign(u) .* abs(u).^(params_new(1));
        idata = iddata(speed',u,dt);
        sys = tfest(idata, 1, 0, params_new(2));
        
        
        if sys.Report.Fit.MSE < mse
            mse = sys.Report.Fit.MSE;
            params = params_new;
            fprintf('%f  %f  %f\n', mse, params(1), params(2));
        end
    end
    
    clf
    hold on
    grid on
    plot(T, speed)
    plot(T, motor_throttle)
    res = lsim(sys,u,T);
    plot(T, res)
    sys = tf(sys);
    
    legend('speed (sensor)', 'throttle', 'speed (model)')
    
    delay_time = params(2);
    input_exponent = params(1);
    PT1_time = sys.Denominator{1}(1) / sys.Denominator{1}(2);
    gain = sys.Numerator{1}(end) / sys.Denominator{1}(end);
    
    fprintf('====================================\n');
    fprintf('          Motor Parameters          \n');
    fprintf('====================================\n');
    fprintf('delay_time      %13f sec\n', delay_time);
    fprintf('input_exponent  %13f\n', input_exponent);
    fprintf('PT1_time        %13f sec\n', PT1_time);
    fprintf('gain            %13f\n', gain);
    
end


function data = load_timeseries(csv_struct, field_name, time_grid_original, time_grid_target)

    data = csv_struct.data(:, strcmp(csv_struct.colheaders, field_name));
    data = interp1(time_grid_original,data,time_grid_target);

end









