function [process_covariance, IPS_covariance] = model_noise_covariance
    
    process_covariance = diag([...
        0.0001... % position deviation [m]
        0.0001... % position deviation [m]
        0.001...% yaw imu offset deviation [rad]
        ].^2);
    
    
    IPS_covariance = diag([...
        0.001... % IPS position deviation [m]
        0.001... % IPS position deviation [m]
        0.005...%  IPS yaw deviation [rad]
        ].^2);
end

