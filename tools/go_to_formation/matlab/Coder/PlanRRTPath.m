function [refPath, isPathValid] = PlanRRTPath (startPose, goalPose, costmap)
    %% Path Planning
        
    startPose = [startPose.x, startPose.y, startPose.yaw];
    goalPose = [goalPose.x, goalPose.y, goalPose.yaw];

    planner = pathPlannerRRT(costmap,...
        'GoalTolerance', [0.01 0.01 3],...
        'MinTurningRadius', 0.4,...
        'ConnectionMethod', 'Dubins');
%         'ConnectionDistance', 0.3);

    [refPath] = plan(planner,startPose,goalPose);
    isPathValid = checkPathValidity(refPath, costmap);

end