function [process_covariance, IPS_covariance] = model_noise_covariance
    
    % Guessed values for standard deviations in 20 ms.
    % Assuming worst-case: Sliding -> model is wrong
    process_covariance = diag([...
        0.005... % position deviation [m]
        0.005... % position deviation [m]
        0.0004...% yaw imu offset deviation [rad]
        ].^2);
    
    
    IPS_covariance = diag([...
        0.0005... % IPS position deviation [m]
        0.0005... % IPS position deviation [m]
        0.0001...%  IPS yaw deviation [rad]
        ].^2);
end

