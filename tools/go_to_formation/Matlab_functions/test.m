
% to be created from VehicleObservation
startPoses = struct('vehicle_1', []);
startPoses.vehicle_1.pose.x = 2.3;
startPoses.vehicle_1.pose.y = 1.5;
startPoses.vehicle_1.pose.yaw = 90; % actually rad

startPoses.vehicle_3 = [];
startPoses.vehicle_3.pose.x = 0.7;
startPoses.vehicle_3.pose.y = 1.5;
startPoses.vehicle_3.pose.yaw = 270; %actually rad

%TODO if input arg when calling script from LCC with arg 'xyz'
allHomePoses = homePosesFixed;
goalPoses = startPoses;
vehicleList = fields(startPoses);
for nVehicles = 1:length(fields(goalPoses))
    goalPoses.(vehicleList{nVehicles}) = allHomePoses(nVehicles);
    goalPoses.(vehicleList{nVehicles}).yaw = 90;
end
%TODO automatic detection of ego vehicle
egoVehicle = 'vehicle_1';
speed = 1; % [m/s]
map = setOccMap(startPoses, egoVehicle);
[refPath, Fig] = PlanAndShowRRTPath(startPoses, goalPoses, egoVehicle, map);
figure(Fig)
trajectory_points = pathToTrajectory(refPath, speed);
pocc = getOccupancy(map, [0.7 1.5])
occupied = checkOccupancy(map, [0.7 1.5])
%TODO: what to do when goalPoses occupied by undesired vehicle?
%Let vehicle be removed by hand or program?
