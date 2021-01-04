function startPoses = readPoses(vehicle_observation_list)

    startPoses = struct('vehicle_id', 0, 'pose', []);

    for nVehicles = 1:length(vehicle_observation_list)
        
        current_id = vehicle_observation_list(1, nVehicles).vehicle_id;
        startPoses(nVehicles).vehicle_id = current_id;
        startPoses(nVehicles).pose = vehicle_observation_list(1,nVehicles).pose; 
        startPoses(nVehicles).pose.yaw = startPoses(nVehicles).pose.yaw * 180 / pi;
        
    end

end


