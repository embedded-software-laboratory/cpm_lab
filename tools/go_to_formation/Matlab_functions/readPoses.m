function startPoses = readPoses(vehicle_observation_list)

startPoses = struct;

for nVehicles = 1:length(vehicle_observation_list)
    currentVehicle = strcat('vehicle_', num2str(vehicle_observation_list(1, nVehicles).vehicle_id));
    startPoses.(currentVehicle) = vehicle_observation_list(1,nVehicles).pose; 
    startPoses.(currentVehicle).yaw = startPoses.(currentVehicle).yaw * 180 / pi;
end
