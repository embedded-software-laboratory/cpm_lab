%% Example for code generation test and variable intitialization

speed = 1; % [m/s] 

startPoses = zeros(3,1);
startPoses(1,1) = 2.1; %[m]
startPoses(2,1) = 2.2; %[m]
startPoses(3,1) = 23.12; %[degree]

goalPose = zeros(3,1);
goalPose(1) = 0.2;
goalPose(2) = 3.6;
goalPose(3) = 90;

[trajectory_points, isPathValid] = planTrajectory(startPoses, 1, goalPose, speed);
[path_pts, is_valid] = plan_path(startPoses, 1, goalPose);
%% Visualization
% This code can be used to visualize generated paths when working 
% on the functions in Matlab. Only paths are shown, not trajectories, as 
% timedependent information is not used.
% To use visualization add 'refPath' and 'planner' as output arguments of
% function planTrajectory. Then add both arguments as outputs of function
% PlanRRTPath, which is called in planTrajectory. 
%
% Fig = 1;
% transitionPoses = interpolate(refPath);
% figure(Fig);
% plot(planner);
% hold on
% scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
%      'DisplayName','Transition Poses')
% hold off