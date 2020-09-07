%function [transitionPoses, costmap, poses] = PlanAndShowRRTPath (startPose, goalPose, xObstacles, yObstacles) %#codegen
%% 
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

% Show the startPose and goalPose positions of the robot
hold on
plot(startPose(1), startPose(2), 'ro')
plot(goalPose(1), goalPose(2), 'mo')

% Show the startPose and goalPose headings
r = 0.1;
plot([startPose(1), startPose(1) + r*cosd(startPose(3))], [startPose(2), startPose(2) + r*sind(startPose(3))], 'r-' )
plot([goalPose(1), goalPose(1) + r*cosd(goalPose(3))], [goalPose(2), goalPose(2) + r*sind(goalPose(3))], 'm-' )
hold off

planner = pathPlannerRRT(costmap,...
    'GoalTolerance', [0.05 0.05 3],...
    'MinTurningRadius', 0.4);


refPath = plan(planner,startPose,goalPose);


nPoses = 100;
lengths = 0 : refPath.Length/nPoses : refPath.Length;
poses = interpolate(refPath,lengths);

nSmoothPoses = 20;
smoothLengths = 0: refPath.Length/nSmoothPoses : refPath.Length;
%[refPoses, refDirections] = interpolate(refPath, smoothLengths);
[refPoses, refDirections] = interpolate(refPath);
transitionPoses = interpolate(refPath);

hold on
plot(costmap)
plot(poses(:,1),poses(:,2), 'DisplayName', 'Path')
scatter(transitionPoses(:,1),transitionPoses(:,2),[],'filled', ...
    'DisplayName','Transition Poses')
hold off
%% % Specify number of poses to return using a separation of approximately 0.1 m
approxSeparation = 0.1; % meters
numSmoothPoses   = round(refPath.Length / approxSeparation);

% Return discretized poses along the smooth path
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(refPoses, refDirections, numSmoothPoses);

% Plot the smoothed path
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

%%  turn obstacle poses into obstacle areas for obstacles being vehicles


