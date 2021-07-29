function startPoses = readPoses(vehicle_observation_list)

    startPoses = struct('vehicle_id', 0, 'pose', []);

    for iVeh = 1:length(vehicle_observation_list)
        
        current_id = vehicle_observation_list(1, iVeh).vehicle_id;
        startPoses(iVeh).vehicle_id = current_id;
        startPoses(iVeh).pose = vehicle_observation_list(1,iVeh).pose; 
        startPoses(iVeh).pose.yaw = startPoses(iVeh).pose.yaw * 180 / pi;
        
    end

end


