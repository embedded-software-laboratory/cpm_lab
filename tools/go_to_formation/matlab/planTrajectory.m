function [trajectory_points, isPathValid] = planTrajectory(vehiclePoses, iEgoVeh, goalPose, speed)
    % vehiclePoses: 3-by-nVeh double
    % iEgoVeh: 1-by-1 uint, index of vehicle to plan trajectory
    % goalPose: 3-by-1 double
    % speed: 1-by-1 double
    % trajectory_points: 5-by-nTrajectoryPoints double
    % isPathValid: bool

    costmap = setCostmap(vehiclePoses, iEgoVeh);
    %% Path Planning
    startPose = vehiclePoses(:,iEgoVeh);
    helpPose = goalPose - 0.6*[cosd(goalPose(3)); sind(goalPose(3)); 0];

    planner = pathPlannerRRT(costmap,...
        'GoalTolerance', [0.01 0.01 3],...
        'MinTurningRadius', 0.5,...
        'ConnectionMethod', 'Dubins', ...
        'MaxIterations', 2000 ...
    );
    
    try
        refPath1 = plan(planner,startPose',helpPose');
        refPath2 = plan(planner,helpPose',goalPose');
        refPath = [refPath1, refPath2]; 
        isPathValid = true;
        trajectory_points = pathToTrajectory(refPath, speed);
    catch
        isPathValid = false;
        trajectory_points = [];
    end
end