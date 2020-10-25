function [trajectory_points] = pathToTrajectory (refPath, speed)

TrajectoryPoint = struct('t', [], 'px', [], 'py', [], 'vx', [], 'vy', []); % single trajectory point
transitionPoses = interpolate(refPath);
trajectory_points = repmat(TrajectoryPoint, length(transitionPoses), 1);
segmentLengths = [];

%%
%TODO: does preallocation save time?
for nPathSegments = 1:length(refPath.PathSegments)
    
    segmentLengths = vertcat(segmentLengths, nonzeros(refPath.PathSegments(1,nPathSegments).MotionLengths));
end

% assert(length(segmentLengths) == length(segmentTypes))
if length(segmentLengths) ~= length(transitionPoses)-1
    disp('no. of segments mismatches no of transition poses')
end


%First TrajectoryPoint 
trajectory_points(1).px = transitionPoses(1, 1);
trajectory_points(1).py = transitionPoses(1, 2);
trajectory_points(1).vx = 0;
trajectory_points(1).vy = 0;
trajectory_points(1).t  = 2e9; %[ns]

trajectory_points(2).px = transitionPoses(1, 1);
trajectory_points(2).py = transitionPoses(1, 2);
trajectory_points(2).vx = cosd(transitionPoses(1, 3)) * speed;
trajectory_points(2).vy = sind(transitionPoses(1, 3)) * speed;
trajectory_points(2).t  = trajectory_points(1).t + 1e9; %[ns]

for nPoints = 3:length(segmentLengths)+2
   
    trajectory_points(nPoints).px = transitionPoses(nPoints-1, 1); % [m]
    trajectory_points(nPoints).py = transitionPoses(nPoints-1, 2); % [m]
    trajectory_points(nPoints).vx = cosd(transitionPoses(nPoints-1, 3)) * speed; % [m/s]
    trajectory_points(nPoints).vy = sind(transitionPoses(nPoints-1, 3)) * speed; % [m/s]
    trajectory_points(nPoints).t  = trajectory_points(nPoints-1).t +(segmentLengths(nPoints-2) / speed * 1e9); % [ns]
    
end

%Last TrajectoryPoint - correct home position guaranteed?
trajectory_points(end+1).px = transitionPoses(end, 1);
trajectory_points(end).py = transitionPoses(end, 2);
trajectory_points(end).vx = 0;
trajectory_points(end).vy = 0;
trajectory_points(end).t  = trajectory_points(end-1).t + 1e9; %[ns]
