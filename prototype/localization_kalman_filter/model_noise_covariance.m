function [process_covariance, internal_covariance, combined_covariance] = model_noise_covariance
    
    % Guessed values for standard deviations in 20 ms.
    % Assuming worst-case: Sliding -> model is wrong
    process_covariance = diag([...
        0.005... % position deviation [m]
        0.005... % position deviation [m]
        0.05...  % yaw deviation [rad]
        0.0004...% yaw imu offset deviation [rad]
        0.005... % position deviation [m]
        0.05...  % speed [m/s]
        0.1...   % curvature [1/m]
        ].^2);
    

    
    internal_covariance = diag([...
        0.05...  % yaw deviation [rad]
        0.005... % position deviation [m]        
        ].^2);
    
    combined_covariance = diag([...
        0.05...  % yaw deviation [rad]
        0.005... % position deviation [m]
        0.0005... % IPS position deviation [m]
        0.0005... % IPS position deviation [m]
        0.0001...%  IPS yaw deviation [rad]
        ].^2);
end

