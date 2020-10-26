function startPoses = readPoses(vehicle_observation_list)

startPoses = struct;

for nVehicles = 1:length(vehicle_observation_list)
    if nVehicles < 10
        currentVehicle = strcat('vehicle_0', num2str(vehicle_observation_list(1, nVehicles).vehicle_id));
    else
        currentVehicle = strcat('vehicle_', num2str(vehicle_observation_list(1, nVehicles).vehicle_id));
    end
    startPoses.(currentVehicle) = vehicle_observation_list(1,nVehicles).pose; 
    startPoses.(currentVehicle).yaw = startPoses.(currentVehicle).yaw * 180 / pi;
end
