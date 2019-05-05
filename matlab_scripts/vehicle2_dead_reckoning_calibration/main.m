function main

    vehicleObservation = csvread('convertedTopic-_vehicleObservation_RecordAll_domain0.csv',2,0);
    vehicleState = csvread('convertedTopic-_vehicleState_RecordAll_domain0.csv',2,0);

    idx_vehicleState_Timestamp                             =  1;
    idx_vehicleState_vehicle_id                            =  2;
    idx_vehicleState_header_create_stamp_nanoseconds       =  3;
    idx_vehicleState_header_valid_after_stamp_nanoseconds  =  4;
    idx_vehicleState_pose_x                                =  5;
    idx_vehicleState_pose_y                                =  6;
    idx_vehicleState_pose_yaw                              =  7;
    idx_vehicleState_IPS_update_age_nanoseconds            =  8;
    idx_vehicleState_odometer_distance                     =  9;
    idx_vehicleState_imu_acceleration_forward              = 10;
    idx_vehicleState_imu_acceleration_left                 = 11;
    idx_vehicleState_imu_yaw                               = 12;
    idx_vehicleState_speed                                 = 13;
    idx_vehicleState_battery_voltage                       = 14;
    idx_vehicleState_motor_current                         = 15;
    idx_vehicleState_motor_throttle                        = 16;
    idx_vehicleState_steering_servo                        = 17;
    
    idx_vehicleObservation_Timestamp                             =  1;
    idx_vehicleObservation_vehicle_id                            =  2;
    idx_vehicleObservation_header_create_stamp_nanoseconds       =  3;
    idx_vehicleObservation_header_valid_after_stamp_nanoseconds  =  4;
    idx_vehicleObservation_pose_x                                =  5;
    idx_vehicleObservation_pose_y                                =  6;
    idx_vehicleObservation_pose_yaw                              =  7;
    
    
    t_vehicleObservation = vehicleObservation(:,idx_vehicleObservation_header_create_stamp_nanoseconds);
    t_vehicleState = vehicleState(:,idx_vehicleState_header_create_stamp_nanoseconds);
    
    %% Match timelines
    [I_state, I_obs] = find(t_vehicleState == t_vehicleObservation');
    vehicleState = vehicleState(I_state,:);
    vehicleObservation = vehicleObservation(I_obs,:);
    
    
    t_vehicleObservation = vehicleObservation(:,idx_vehicleObservation_header_create_stamp_nanoseconds);
    t_vehicleState = vehicleState(:,idx_vehicleState_header_create_stamp_nanoseconds);
    assert(all(t_vehicleObservation == t_vehicleState));
    
    clf
    hold on
    plot(vehicleObservation(:,idx_vehicleObservation_pose_yaw))
    plot(vehicleState(:,idx_vehicleState_imu_yaw)+2.29)
    
end

