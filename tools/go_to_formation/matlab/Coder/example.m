%% Example for code generation test and variable intitialization

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
    if (iVeh == 1)
        startPoses(iVeh).pose.x = 3.88121761206338; %[m]
        startPoses(iVeh).pose.y = 3.72232737479556; %[m]
        startPoses(iVeh).pose.yaw = 360-28.7279528555192; %[degree]
    end
end

goalPose = Pose2D;
goalPose.x = 0.2;
goalPose.y = 3.6;
goalPose.yaw = 90;

[trajectory_points, isPathValid, planner, refPath] = planTrajectory(maxVehicleIdList, startPoses, goalPose, egoVehicleId, speed);
%% Visualization
% This code can be used to visualize generated paths when working 
% on the functions in Matlab. Only paths are shown, not trajectories, as 
% timedependent information is not used.
% To use visualization add 'refPath' and 'planner' as output arguments of
% function planTrajectory. Then add both arguments as outputs of function
% PlanRRTPath, which is called in planTrajectory. 
%
Fig = 1;
transitionPoses = interpolate(refPath);
figure(Fig);
plot(planner);
hold on
scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
     'DisplayName','Transition Poses')
hold off