%% Example for code generation test

load('test_poses.mat', 'sample')
startPoses = readPoses(sample.vehicle_observation_list);
goalPoses = homePosesFixed();
egoVehicle = 'vehicle_01';
vehicleList = {'vehicle_01', 'vehicle_02', 'vehicle_03'};
speed = 1; % [m/s] 
trajectory_points = plan_trajectory(vehicleList, startPoses, goalPoses(1), egoVehicle, speed);
save('trajector_points.mat', 'trajectory_points')
%% Visualization

% Fig = 1;
% transitionPoses = interpolate(refPath);
% figure(Fig);
% plot(planner);
% hold on
% scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
%      'DisplayName','Transition Poses')
% hold off