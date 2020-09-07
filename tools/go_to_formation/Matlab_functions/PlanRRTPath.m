function [transitionPoses] = PlanRRTPath (startPose, goalPose, xObstacles, yObstacles) %#codegen
%% Setup of Costmap  
mapX = 4.5;
mapY = 4.0;
costVal = 0;
cellSize = 0.05;
occupiedVal = 0.9;

centerPlacements = [0.25, 0.5, 0.75];
inflationRadius = 0.08;

vehLength = 0.22;
vehicleDims = vehicleDimensions(vehLength,0.107, 0.07,...
    'FrontOverhang', 0.04,...
    'RearOverhang',0.03);

ccConfig = inflationCollisionChecker(vehicleDims, ...
    'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

costmap = vehicleCostmap(mapX,mapY,costVal,...
    'CellSize',cellSize,...
    'CollisionChecker',ccConfig);

[xMapLimits, yMapLimits] = SetMapLimits(mapX, mapY, cellSize);
setCosts(costmap, [xMapLimits, yMapLimits], occupiedVal);
setCosts(costmap,[xObstacles, yObstacles],occupiedVal)

%% Path Planning and Visualization

planner = pathPlannerRRT(costmap,...
    'GoalTolerance', [0.05 0.05 3],...
    'MinTurningRadius', 0.4);

refPath = plan(planner,startPose,goalPose);
transitionPoses = interpolate(refPath);