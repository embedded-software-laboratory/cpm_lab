%% Example for code generation test and variable intitialization

% load('test_poses.mat', 'sample')
% startPoses = readPoses(sample.vehicle_observation_list);

% Import IDL files from cpm library
dds_idl_matlab = fullfile(getenv('HOME'),'dev/software/cpm_lib/dds_idl_matlab/');
assert(isfolder(dds_idl_matlab),...
    'Missing directory "%s".', dds_idl_matlab);
assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
    'No MATLAB IDL-files found in %s', dds_idl_matlab);
addpath(dds_idl_matlab)

goalPoses = homePosesFixed();
egoVehicleId = uint8(1);
maxVehicleIdList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
speed = 1; % [m/s] 
startPoses = struct;
for iVeh = maxVehicleIdList
    startPoses(iVeh).vehicle_id = cast(iVeh, 'uint8');
    startPoses(iVeh).pose = Pose2D;
    if (iVeh<4)
        startPoses(iVeh).pose.x = 2;
        startPoses(iVeh).pose.y = 0.5+iVeh*0.5;
        startPoses(iVeh).pose.yaw = 0;
    end
end

[trajectory_points, isPathValid] = planTrajectory(maxVehicleIdList, startPoses, goalPoses(1), egoVehicleId, speed);
%% Visualization
% 
% Fig = 1;
% transitionPoses = interpolate(refPath);
% figure(Fig);
% plot(planner);
% hold on
% scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
%      'DisplayName','Transition Poses')
% hold off