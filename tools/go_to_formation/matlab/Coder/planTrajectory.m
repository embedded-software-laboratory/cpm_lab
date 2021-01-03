function [trajectory_points] = planTrajectory(vehicleList, vehiclePoses, goalPose, egoVehicle, speed)

    map = setOccMap(vehicleList, vehiclePoses, egoVehicle);
    [refPath, isPathValid, ~] = PlanRRTPath(vehiclePoses.(egoVehicle), goalPose, map);

    if isPathValid
        trajectory_points = pathToTrajectory(refPath, speed);
    else
        trajectory_points = 0;
    end

end