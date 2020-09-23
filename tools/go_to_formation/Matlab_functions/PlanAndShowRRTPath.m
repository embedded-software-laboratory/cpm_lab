function [transitionPoses, costmap, poses] = PlanAndShowRRTPath (startPoses, goalPoses) %#codegen

egoVehicle = 'vehicle_1';

%% Setup of Costmap and Planner Configuration

centerPlacements = [0.25, 0.5, 0.75];
inflationRadius = 0.05;
vehLength = 0.22;
vehicleDims = vehicleDimensions(vehLength,0.107, 0.07,...
    'FrontOverhang', 0.04,...
    'RearOverhang',0.03);

ccConfig = inflationCollisionChecker(vehicleDims, ...
    'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

occMap = setOccMap(startPoses, egoVehicle);
costmap = vehicleCostmap(occMap, 'CollisionChecker', ccConfig);

%% Path Planning and Visualization

startPose = [startPoses.vehicle_1.x, startPoses.vehicle_1.y, startPoses.vehicle_1.yaw];
goalPose = [goalPoses.vehicle_1.x, goalPoses.vehicle_1.y, goalPoses.vehicle_1.yaw];

planner = pathPlannerRRT(costmap,...
    'GoalTolerance', [0.05 0.05 3],...
    'MinTurningRadius', 0.4);


refPath = plan(planner,startPose,goalPose);
nPoses = 100;
lengths = 0 : refPath.Length/nPoses : refPath.Length;
poses = interpolate(refPath,lengths);

[refPoses, refDirections] = interpolate(refPath);
transitionPoses = interpolate(refPath);


plot(costmap)
hold on
plot(poses(:,1),poses(:,2), 'DisplayName', 'Path')
scatter(transitionPoses(:,1),transitionPoses(:,2),[],'filled', ...
    'DisplayName','Transition Poses')
hold off

