function [refPath, isPathValid] = PlanAndShowRRTPath (startPose, goalPose, occMap)
%% Setup of Costmap and Planner Configuration

    centerPlacements = [0.25, 0.5, 0.75]; 
    vehLength = 0.22;
    vehicleDims = vehicleDimensions(vehLength,0.107, 0.07,...
        'FrontOverhang', 0.04,...
        'RearOverhang',0.03);
    inflationRadius = 0;
    ccConfig = inflationCollisionChecker(vehicleDims, ...
        'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

    costmap = vehicleCostmap(occMap, 'CollisionChecker', ccConfig);

    %% Path Planning

    startPose = [startPose.x, startPose.y, startPose.yaw];
    goalPose = [goalPose.x, goalPose.y, goalPose.yaw];

    planner = pathPlannerRRT(costmap,...
        'GoalTolerance', [0.01 0.01 3],...
        'MinTurningRadius', 0.4,...
        'ConnectionMethod', 'Dubins');

    try 
        [refPath] = plan(planner,startPose,goalPose);
        isPathValid = checkPathValidity(refPath, costmap);

    %     Visualization: uncomment when needed
    %     Fig = figure;
    %     plot(planner);
    %     hold on
    %     scatter(morePoses(:, 1), morePoses(:,2), [], 'filled')
    %     hold on
    %     scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
    %          'DisplayName','Transition Poses')
    %     hold off

    catch
        disp('Not able to plan valid path')
        isPathValid = false;
        refPath = 0;
    end


end