%% Example for code generation test and variable intitialization

load('test_poses.mat', 'sample')
startPoses = readPoses(sample.vehicle_observation_list);
goalPoses = homePosesFixed();
egoVehicleId = 1;
vehicleIdList = [1,2,3];
maxVehicleIdList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
speed = 1; % [m/s] 
for missingVehicles = 4:20
    startPoses(missingVehicles).vehicle_id = cast(missingVehicles, 'uint8');
    startPoses(missingVehicles).pose = Pose2D;
end

[trajectory_points,refPath,planner] = planTrajectory(maxVehicleIdList, startPoses, goalPoses(1), egoVehicleId, speed);
%% Visualization

Fig = 1;
transitionPoses = interpolate(refPath);
figure(Fig);
plot(planner);
hold on
scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
     'DisplayName','Transition Poses')
hold off