function [refPath, Fig] = PlanAndShowRRTPath (startPoses, goalPoses, egoVehicle, occMap) %#codegen
%% Setup of Costmap and Planner Configuration

centerPlacements = [0.25, 0.5, 0.75]; 
vehLength = 0.22;
vehicleDims = vehicleDimensions(vehLength,0.107, 0.07,...
    'FrontOverhang', 0.04,...
    'RearOverhang',0.03);
inflationRadius = 0; % wouldnt half vehicle width be better?
ccConfig = inflationCollisionChecker(vehicleDims, ...
    'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

%occMap = setOccMap(startPoses, egoVehicle);
costmap = vehicleCostmap(occMap, 'CollisionChecker', ccConfig);

%% Path Planning

%TODO: replace by input arguments and vehicles pose readout
startPose = [startPoses.(egoVehicle).x, startPoses.(egoVehicle).y, startPoses.(egoVehicle).yaw];
goalPose = [goalPoses.(egoVehicle).x, goalPoses.(egoVehicle).y, goalPoses.(egoVehicle).yaw];

% if(startPoses.(egoVehicle).pose == goalPoses.(egoVehicle))
%     disp('Vehicle already in goal position')
% end

planner = pathPlannerRRT(costmap,...
    'GoalTolerance', [0.01 0.01 3],...
    'MinTurningRadius', 0.4,...
    'ConnectionMethod', 'Dubins');

[refPath] = plan(planner,startPose,goalPose);

%% Visualization

transitionPoses = interpolate(refPath);
Fig = figure;
plot(planner);
hold on
scatter(transitionPoses(1:end-1,1),transitionPoses(1:end-1,2),[],'filled', ...
     'DisplayName','Transition Poses')
hold off