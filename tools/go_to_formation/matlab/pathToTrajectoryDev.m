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
    

end