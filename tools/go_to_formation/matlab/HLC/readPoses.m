function startPoses = readPoses(vehicle_observation_list)

    startPoses = struct;

    for nVehicles = 1:length(vehicle_observation_list)

        current_id = num2str(vehicle_observation_list(1, nVehicles).vehicle_id);
        if numel(current_id) < 2
            currentVehicle = strcat('vehicle_0', current_id);
        else
            currentVehicle = strcat('vehicle_', current_id);
        end

        startPoses.(currentVehicle) = vehicle_observation_list(1,nVehicles).pose; 
        startPoses.(currentVehicle).yaw = startPoses.(currentVehicle).yaw * 180 / pi;
    end

end


