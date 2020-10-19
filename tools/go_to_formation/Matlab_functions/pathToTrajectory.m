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

%First TrajectoryPoint 
trajectory_points(1).px = transitionPoses(1, 1);
trajectory_points(1).py = transitionPoses(1, 2);
trajectory_points(1).vx = 0;
trajectory_points(1).vy = 0;
trajectory_points(1).t  = 10e9; %[ns]

% trajectory_points(2).px = transitionPoses(1, 1);
% trajectory_points(2).py = transitionPoses(1, 2);
% trajectory_points(2).vx = 0;
% trajectory_points(2).vy = 0;
% trajectory_points(2).t  = 10e9; %[ns]
% 
% for nTrajectoryPoints = 3:length(segmentLengths)+1
%    
%     trajectory_points(nTrajectoryPoints).px = transitionPoses(nTrajectoryPoints-1, 1); % [m]
%     trajectory_points(nTrajectoryPoints).py = transitionPoses(nTrajectoryPoints-1, 2); % [m]1
%     trajectory_points(nTrajectoryPoints).vx = cosd(transitionPoses(nTrajectoryPoints-1, 3)) * speed; % [m/s]
%     trajectory_points(nTrajectoryPoints).vy = sind(transitionPoses(nTrajectoryPoints-1, 3)) * speed; % [m/s]
%     trajectory_points(nTrajectoryPoints).t  = uint64(segmentLengths(nTrajectoryPoints-2) / speed * 10e9); % [ns]
%     
% end


for nTrajectoryPoints = 2:length(segmentLengths)+1
   
    trajectory_points(nTrajectoryPoints).px = transitionPoses(nTrajectoryPoints, 1); % [m]
    trajectory_points(nTrajectoryPoints).py = transitionPoses(nTrajectoryPoints, 2); % [m]1
    trajectory_points(nTrajectoryPoints).vx = cosd(transitionPoses(nTrajectoryPoints, 3)) * speed; % [m/s]
    trajectory_points(nTrajectoryPoints).vy = sind(transitionPoses(nTrajectoryPoints, 3)) * speed; % [m/s]
    trajectory_points(nTrajectoryPoints).t  = uint64(segmentLengths(nTrajectoryPoints-1) / speed * 10e9); % [ns]
    
end
%Last TrajectoryPoint - correct home position guaranteed?
trajectory_points(end).px = transitionPoses(end, 1);
trajectory_points(end).py = transitionPoses(end, 2);
trajectory_points(end).vx = 0;
trajectory_points(end).vy = 0;
trajectory_points(end).t  = 10e10; %[ns]
