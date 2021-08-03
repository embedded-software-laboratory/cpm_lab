function [trajectory_points] = pathToTrajectory (refPath, speed)
    % trajectory_points: 5-by-nTrajPts double
    % refPath has two entries:
    % first one leads to the pose in front of goal pose,
    % second one is straight line to goal pose

    %% Calculate path lengths for all poses allPathLenghts
    % interpolate intermitting poses for more exact translation into spline.
    ds = 0.1;
    allPoses = [];
    transitionPoses = [];
    transSegmentLengths = [];
    lengths = [];
    for irp = 1:numel(refPath)
        rp = refPath(irp);
        curLengths = ds:ds:rp.Length; % Path length between two intermitting poses.
        addAllPoses = interpolate(rp,curLengths);
        if isempty(lengths)
            lengths = curLengths;
        else
            lengths = [lengths, sum(transSegmentLengths)+curLengths]; %#ok<AGROW>
        end
        addTransitionPoses = interpolate(rp);
        if irp>1
            % remove double poses on concatenation
            addAllPoses = addAllPoses(2:end,:);
            addTransitionPoses = addTransitionPoses(2:end,:);
        end
        allPoses = [allPoses; addAllPoses]; %#ok<AGROW>
        transitionPoses = [transitionPoses; addTransitionPoses]; %#ok<AGROW>
        for nPathSegments = 1:length(rp.PathSegments)
            motionLengths = nonzeros(rp.PathSegments(nPathSegments).MotionLengths);
            transSegmentLengths = [transSegmentLengths; motionLengths]; %#ok<AGROW>
        end
    end
    % Merge transition poses with intermitting poses into vector allSegmentLengths
    checkPose = ismember(allPoses, transitionPoses);
    isTransPose = all(checkPose, 2);
    transPathLengths = [0; cumsum(transSegmentLengths)];
    allPathLengths = zeros(length(allPoses), 1);
    allPathLengths(isTransPose) = transPathLengths;
    allPathLengths(~isTransPose) = lengths;

    %% Set speed profile
    a = 1; % m/s^2
    allSpeeds = min(sqrt(allPathLengths*a), sqrt( (allPathLengths(end)-allPathLengths)*a ));
    allSpeeds(allSpeeds>speed) = speed;
    vx = cosd(allPoses(:, 3)) .* allSpeeds;
    vy = sind(allPoses(:, 3)) .* allSpeeds;
    
    %% Calculate timestamps
    allSegmentLengths = diff(allPathLengths);
    avgSegmentSpeed = 0.5 * (allSpeeds(1:end-1) + allSpeeds(2:end));
    segmentDurations = allSegmentLengths./avgSegmentSpeed;
    timestamps = uint64([0; cumsum(segmentDurations)*1e9]);

    %% Fill trajectory points
    nTrajPts = numel(timestamps);
    trajectory_points = zeros(5,nTrajPts);
    for iPt = 1:nTrajPts
        trajectory_points(1,iPt) = allPoses(iPt, 1);
        trajectory_points(2,iPt) = allPoses(iPt, 2);
        trajectory_points(3,iPt) = vx(iPt);
        trajectory_points(4,iPt) = vy(iPt);
        trajectory_points(5,iPt) = timestamps(iPt); %[ns]
    end
end
