function [trajectory_points, isPathValid] = planTrajectory(vehicleIdList, vehiclePoses, goalPose, egoVehicleId, speed)
    
    egoVehicleIndex = 0;
    for nVehicles = 1:length(vehicleIdList)
        if vehiclePoses(nVehicles).vehicle_id == egoVehicleId
            egoVehicleIndex = nVehicles;
            break
        end
    end  
                
    costmap = setCostmap(vehiclePoses, egoVehicleId);
    startPose = vehiclePoses(egoVehicleIndex).pose;
    %% Path Planning
    startPoseVec = [startPose.x, startPose.y, startPose.yaw];
    goalPoseVec = [goalPose.x, goalPose.y, goalPose.yaw];
    a = goalPose.yaw;
    helpPoseVec = goalPoseVec - 0.6*[cosd(a) sind(a) 0];

    planner = pathPlannerRRT(costmap,...
        'GoalTolerance', [0.01 0.01 3],...
        'MinTurningRadius', 0.5,...
        'ConnectionMethod', 'Dubins', ...
        'MaxIterations', 2000 ...
    );

    try % not supported by code generation
        refPath1 = plan(planner,startPoseVec,helpPoseVec);
        refPath2 = plan(planner,helpPoseVec,goalPoseVec);
        refPath = [refPath1, refPath2]; % not supported by code generation
        isPathValid = true;
        trajectory_points = pathToTrajectory(refPath, speed);
    catch
        isPathValid = false;
        TrajectoryPoint = struct('t', uint64(0), 'px', 0, 'py', 0, 'vx', 0, 'vy', 0);
        trajectory_points = TrajectoryPoint;
    end
end