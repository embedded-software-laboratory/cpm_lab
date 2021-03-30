function [trajectory_points] = pathToTrajectory (refPath, travelling_speed)

    transitionPoses = interpolate(refPath);
    morePoses = interpolate(refPath, 0.3);
    
    
    TrajectoryPoint = struct('t', uint64(0), 'px', 0, 'py', 0, 'vx', 0, 'vy', 0); % single trajectory point

    trajectory_points = repmat(TrajectoryPoint, length(transitionPoses), 1);
    
    nTrajectoryPoints = length(transitionPoses);
    nPathSegments = length(refPath.PathSegments);
    segmentLength = zeros(1:transitionPoses-1);
    
    for i = 1:nPathSegments
        
        for j = 1:3 %Dubins Path always consists of three elements
            segmentLength((i-1)*3+j) = refPath.PathSegments(1,i).MotionLengths(j);
        end
        
    end
   
    segmentLength = nonzeros(segmentLength);
    counter = 0;
    for k = 1:length(transitionPoses)-1
        if all((transitionPoses(k,:)-transitionPoses(k+1,:)) < 1e-5)
            counter = counter+1;
        end
    end
    
    smallSegmentCounter = 0;
    for l = 1:length(segmentLength)
        if segmentLength(l) < 1e-5
            smallSegmentCounter = smallSegmentCounter+1;
        end
    end
    
    %% 

    trajectory_points(i).px = transitionPoses(1, 1);
    trajectory_points(i).py = transitionPoses(1, 2);
    trajectory_points(i).vx = 0;
    trajectory_points(i).vy = 0;
    trajectory_points(i).t  = 1e9; %[ns]
    
    for i = 2:nTrajectoryPoints
    
        if nTrajectoryPoints >= 10

            speed = min(i/5.0, travelling_speed,0  );   
            
            trajectory_points(i).px = transitionPoses(i, 1);
            trajectory_points(i).py = transitionPoses(i, 2);
            trajectory_points(i).vx = cosd(transitionPoses(i, 3)) * speed;
            trajectory_points(i).vy = sind(transitionPoses(i, 3)) * speed;
            trajectory_points(i).t  = uint64(trajectory_points(i-1).t) +...
                                         (transitionPoses(nPoints-1) / speed * 1e9);
            

        else


        end
        
    end
    

    
    
%     % Additionally to transition poses of path segments 
%     % interpolate intermitting poses for more exact translation into spline.
%     lengths = 0 : 0.5 : refPath.Length; % Path length between two intermitting poses.
%     intermittingPoses = interpolate(refPath,lengths); 
%     transitionPoses = interpolate(refPath);
%  
%     %% Calculate path lengths between transition and intermitting poses
%     transSegmentLengths = zeros(100, 1);
%     allSegmentLengths = zeros(length(intermittingPoses), 1);
%     lastIndex = 0;
%     helpIndex = [];
%     for nPathSegments = 1:length(refPath.PathSegments)
%         motionLengths = nonzeros(refPath.PathSegments(nPathSegments).MotionLengths);
%         transSegmentLengths(lastIndex+1:lastIndex+length(motionLengths), 1) = motionLengths;
%         helpIndex = find(transSegmentLengths, 1, 'last');
%         lastIndex = helpIndex(end);
%     end
%     transSegmentLengths = nonzeros(transSegmentLengths);
%     transPathLengths = cumsum(transSegmentLengths);
%     
%     checkPose = ismember(intermittingPoses, transitionPoses);
%     isTransPose = all(checkPose, 2);
% 
%     % Merge transition poses with intermitting poses into vector allSegmentLengths
%     allSegmentLengths(1) = 0;
%     for nPoses = 2:length(intermittingPoses)-1
%         if isTransPose(nPoses)
%             index = sum(isTransPose(1:nPoses));
%             allSegmentLengths(nPoses) = transPathLengths(index-1);
%         else
%             noTransPoses = sum(isTransPose(1:nPoses));
%             allSegmentLengths(nPoses) = (nPoses-noTransPoses) * 0.5;
%         end
%     end
% 
%     allSegmentLengths(end) = transPathLengths(end);
% 
%     %% Set trajectory points from transition and intermitting poses
%     %TODO: smoothing of start and stop process.
%      trajectory_points = repmat(TrajectoryPoint, length(allSegmentLengths), 1);
%     
%     %First trajectory point 
%     trajectory_points(1).px = intermittingPoses(1, 1);
%     trajectory_points(1).py = intermittingPoses(1, 2);
%     trajectory_points(1).vx = 0;
%     trajectory_points(1).vy = 0;
%     trajectory_points(1).t  = uint64(1e9); %[ns]
% 
%     % Second trajectory point
%     trajectory_points(2).px = intermittingPoses(1, 1);
%     trajectory_points(2).py = intermittingPoses(1, 2);
%     trajectory_points(2).vx = cosd(intermittingPoses(1, 3)) * speed;
%     trajectory_points(2).vy = sind(intermittingPoses(1, 3)) * speed;
%     trajectory_points(2).t  = uint64(trajectory_points(1).t + 1e9); %[ns]
% 
%     for nPoints = 3:length(allSegmentLengths)
% 
%         trajectory_points(nPoints).px = intermittingPoses(nPoints-1, 1); % [m]
%         trajectory_points(nPoints).py = intermittingPoses(nPoints-1, 2); % [m]
%         trajectory_points(nPoints).vx = cosd(intermittingPoses(nPoints-1, 3)) * speed; % [m/s]
%         trajectory_points(nPoints).vy = sind(intermittingPoses(nPoints-1, 3)) * speed; % [m/s]
%         trajectory_points(nPoints).t  = uint64(trajectory_points(2).t) +...
%                                         (allSegmentLengths(nPoints-1) / speed * 1e9); % [ns]
%     end
% 
%     %Second Last TrajectoryPoint
%     trajectory_points(end-1).px = intermittingPoses(end, 1);
%     trajectory_points(end-1).py = intermittingPoses(end, 2);
%     trajectory_points(end-1).vx = 0;
%     trajectory_points(end-1).vy = 1;
%     trajectory_points(end-1).t  = uint64(trajectory_points(2).t) +...
%                                 (allSegmentLengths(end) / speed * 1e9); %[ns]
%     %Last Trajectory Point
%     trajectory_points(end).px = intermittingPoses(end, 1);
%     trajectory_points(end).py = intermittingPoses(end, 2);
%     trajectory_points(end).vx = 0;
%     trajectory_points(end).vy = 0;
%     trajectory_points(end).t  = uint64(trajectory_points(end-1).t + 1e8); %[ns]

end
