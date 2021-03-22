function [trajectory_points, isPathValid] = planTrajectory(vehicleIdList, vehiclePoses, goalPose, egoVehicleId, speed)
    
    egoVehicleIndex = 0;
    for nVehicles = 1:length(vehicleIdList)
        if vehiclePoses(nVehicles).vehicle_id == egoVehicleId
            egoVehicleIndex = nVehicles;
            break
        end
    end  
                
    costmap = setCostmap(vehiclePoses, egoVehicleId);
    [refPath, isPathValid] = PlanRRTPath(vehiclePoses(egoVehicleIndex).pose, goalPose, costmap);

    if isPathValid
       trajectory_points = pathToTrajectory(refPath, speed);
    else
        TrajectoryPoint = struct('t', uint64(0), 'px', 0, 'py', 0, 'vx', 0, 'vy', 0);
        trajectory_points = TrajectoryPoint;
    end

end