function [trajectory_points, refPath, planner] = planTrajectory(vehicleIdList, vehiclePoses, goalPose, egoVehicleId, speed)
    
    egoVehicleIndex = 0;
    for nVehicles = 1:length(vehicleIdList)
        if vehiclePoses(nVehicles).vehicle_id == egoVehicleId
            egoVehicleIndex = nVehicles;
            break
        end
    end  
                
    map = setOccMap(vehiclePoses, egoVehicleId);
    [refPath, isPathValid, planner] = PlanRRTPath(vehiclePoses(egoVehicleIndex).pose, goalPose, map);

    if isPathValid
        trajectory_points = pathToTrajectory(refPath, speed);
    else
        trajectory_points = 0;
    end

end