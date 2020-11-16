function [trajectory_points] = pathToTrajectory (refPath, speed)

TrajectoryPoint = struct('t', [], 'px', [], 'py', [], 'vx', [], 'vy', []); % single trajectory point
lengths = 0 : 0.5 : refPath.Length;
morePoses = interpolate(refPath,lengths); 
save('morePoses.mat', 'morePoses')
transitionPoses = interpolate(refPath);
save('transitionPoses.mat', 'transitionPoses')
trajectory_points = repmat(TrajectoryPoint, length(transitionPoses), 1);
segmentLengths = [];
for nPathSegments = 1:length(refPath.PathSegments)
    segmentLengths = vertcat(segmentLengths, nonzeros(refPath.PathSegments(1,nPathSegments).MotionLengths));
end
%%
%allSegmentLengths = refPath.PathSegments(1).MotionLengths(1);
transSegmentLengths = [];

% TODO: does preallocation save time?
for nPathSegments = 1:length(refPath.PathSegments)
    transSegmentLengths = vertcat(transSegmentLengths, nonzeros(refPath.PathSegments(1,nPathSegments).MotionLengths));
end

transPathLengths = cumsum(transSegmentLengths);
checkPose = ismember(morePoses, transitionPoses);
isTransPose = all(checkPose, 2);

allSegmentLengths(1) = 0;

for nPoses = 2:length(morePoses)-1
    if isTransPose(nPoses)
        index = sum(isTransPose(1:nPoses));
        allSegmentLengths(nPoses) = transPathLengths(index-1);
    else
        noTransPoses = sum(isTransPose(1:nPoses));
        allSegmentLengths(nPoses) = (1+length(allSegmentLengths)-noTransPoses) * 0.5;
    end
    
end

allSegmentLengths(end+1) = transPathLengths(end);


%% 
%First trajectory point 
trajectory_points(1).px = morePoses(1, 1);
trajectory_points(1).py = morePoses(1, 2);
trajectory_points(1).vx = 0;
trajectory_points(1).vy = 0;
trajectory_points(1).t  = 2e9; %[ns]

% Second trajectory point
trajectory_points(2).px = morePoses(1, 1);
trajectory_points(2).py = morePoses(1, 2);
trajectory_points(2).vx = cosd(morePoses(1, 3)) * speed;
trajectory_points(2).vy = sind(morePoses(1, 3)) * speed;
trajectory_points(2).t  = trajectory_points(1).t + 1e9; %[ns]

for nPoints = 3:length(allSegmentLengths)
   
    trajectory_points(nPoints).px = morePoses(nPoints-1, 1); % [m]
    trajectory_points(nPoints).py = morePoses(nPoints-1, 2); % [m]
    trajectory_points(nPoints).vx = cosd(morePoses(nPoints-1, 3)) * speed; % [m/s]
    trajectory_points(nPoints).vy = sind(morePoses(nPoints-1, 3)) * speed; % [m/s]
    trajectory_points(nPoints).t  = trajectory_points(2).t +...
                                    allSegmentLengths(nPoints-1) / speed * 1e9; % [ns]
    %trajectory_points(nPoints).t  = trajectory_points(nPoints-1).t +  / speed * 1e9)
    
end

%Last TrajectoryPoint - correct home position guaranteed?
trajectory_points(end+1).px = morePoses(end, 1);
trajectory_points(end).py = morePoses(end, 2);
trajectory_points(end).vx = 0;
trajectory_points(end).vy = 0;
trajectory_points(end).t  = trajectory_points(end-1).t + 1e9; %[ns]

end
