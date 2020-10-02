function [trajectory_points] = pathToTrajectory (refPath, speed)

TrajectoryPoint = struct('t', [], 'px', [], 'py', [], 'vx', [], 'vy', []); % single trajectory point
transitionPoses = interpolate(refPath);
trajectory_points = repmat(TrajectoryPoint, length(transitionPoses), 1);
segmentLengths = [];

% segmentTypes = [];
% validSegmentTypes = ["L"; "R"; "S"];
%%
%TODO: does preallocation save time?
for nPathSegments = 1:length(refPath.PathSegments)
    
    segmentLengths = vertcat(segmentLengths, nonzeros(refPath.PathSegments(1,nPathSegments).MotionLengths));
    
%     currentSegmentTypes = refPath.PathSegments(1,nPathSegments).MotionTypes;
%     validSegments = contains(currentSegmentTypes, validSegmentTypes);
%     segmentTypes = vertcat(segmentTypes, currentSegmentTypes(validSegments)');
end

% assert(length(segmentLengths) == length(segmentTypes))

assert(length(segmentLengths) == length(transitionPoses)-1)
 % array of structs, TrajectoryPoint sequence

for nTrajectoryPoints = 1:length(segmentLengths)
   
    trajectory_points(nTrajectoryPoints).px = transitionPoses(nTrajectoryPoints, 1); % [m]
    trajectory_points(nTrajectoryPoints).py = transitionPoses(nTrajectoryPoints, 2); % [m]
    trajectory_points(nTrajectoryPoints).vx = cosd(transitionPoses(nTrajectoryPoints, 3)) * speed; % [m/s]
    trajectory_points(nTrajectoryPoints).vy = sind(transitionPoses(nTrajectoryPoints, 3)) * speed; % [m/s]
    % TODO time in nanoseconds
    trajectory_points(nTrajectoryPoints).t  = segmentLengths(nTrajectoryPoints) / speed * 1000000; % [ns]
    
end

%Last TrajectoryPoint - correct home position guaranteed?
trajectory_points(end).px = transitionPoses(end, 1);
trajectory_points(end).py = transitionPoses(end, 2);
trajectory_points(end).vx = 0;
trajectory_points(end).vy = 0;
trajectory_points(end).t  = 500000; %[ns]
