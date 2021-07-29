function [refPath, isPathValid] = PlanRRTPath (startPose, goalPose, costmap)
    %% Path Planning
    refPath = [];
    isPathValid = true;
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
    catch
        isPathValid = false;
    end
end