function [trajectory_points] = pathToTrajectory (refPath, speed)

    TrajectoryPoint = struct('t', [], 'px', [], 'py', [], 'vx', [], 'vy', []); % single trajectory point
    
    % Additionally to transition poses of path segments 
    % interpolate intermitting poses for more exact translation into spline.
    lengths = 0 : 0.5 : refPath.Length; % Path length between two intermitting poses.
    intermittingPoses = interpolate(refPath,lengths); 
    transitionPoses = interpolate(refPath);
 
    trajectory_points = repmat(TrajectoryPoint, length(transitionPoses), 1);
    %% Calculate path lengths between transition and intermitting poses
    transSegmentLengths = [];
    % TODO: does preallocation save time?
    for nPathSegments = 1:length(refPath.PathSegments)
        transSegmentLengths = vertcat(transSegmentLengths, nonzeros(refPath.PathSegments(1,nPathSegments).MotionLengths));
    end
    transPathLengths = cumsum(transSegmentLengths);
    
    checkPose = ismember(intermittingPoses, transitionPoses);
    isTransPose = all(checkPose, 2);

    % Merge transition poses with intermitting poses into vector allSegmentLengths
    allSegmentLengths(1) = 0;
    for nPoses = 2:length(intermittingPoses)-1
        if isTransPose(nPoses)
            index = sum(isTransPose(1:nPoses));
            allSegmentLengths(nPoses) = transPathLengths(index-1);
        else
            noTransPoses = sum(isTransPose(1:nPoses));
            allSegmentLengths(nPoses) = (1+length(allSegmentLengths)-noTransPoses) * 0.5;
        end
    end

    allSegmentLengths(end+1) = transPathLengths(end);

    %% Set trajectory points from transition and intermitting poses
    %TODO: smoothing of start and stop process.
    
    %First trajectory point 
    trajectory_points(1).px = intermittingPoses(1, 1);
    trajectory_points(1).py = intermittingPoses(1, 2);
    trajectory_points(1).vx = 0;
    trajectory_points(1).vy = 0;
    trajectory_points(1).t  = 2e9; %[ns]

    % Second trajectory point
    trajectory_points(2).px = intermittingPoses(1, 1);
    trajectory_points(2).py = intermittingPoses(1, 2);
    trajectory_points(2).vx = cosd(intermittingPoses(1, 3)) * speed;
    trajectory_points(2).vy = sind(intermittingPoses(1, 3)) * speed;
    trajectory_points(2).t  = trajectory_points(1).t + 1e9; %[ns]

    for nPoints = 3:length(allSegmentLengths)

        trajectory_points(nPoints).px = intermittingPoses(nPoints-1, 1); % [m]
        trajectory_points(nPoints).py = intermittingPoses(nPoints-1, 2); % [m]
        trajectory_points(nPoints).vx = cosd(intermittingPoses(nPoints-1, 3)) * speed; % [m/s]
        trajectory_points(nPoints).vy = sind(intermittingPoses(nPoints-1, 3)) * speed; % [m/s]
        trajectory_points(nPoints).t  = trajectory_points(2).t +...
                                        allSegmentLengths(nPoints-1) / speed * 1e9; % [ns]
    end

    %Last TrajectoryPoint - correct home position guaranteed?
    trajectory_points(end+1).px = intermittingPoses(end, 1);
    trajectory_points(end).py = intermittingPoses(end, 2);
    trajectory_points(end).vx = 0;
    trajectory_points(end).vy = 0;
    trajectory_points(end).t  = trajectory_points(end-1).t + 1e9; %[ns]

end
