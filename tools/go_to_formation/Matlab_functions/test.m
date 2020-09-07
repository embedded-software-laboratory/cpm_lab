startPose = [0.3, 3.7, -90]; % [meters, meters, degrees]
goalPose  = [2.0, 1.5, 90];

xObstacles = [0.3, 0.4, 0.5, 0.6, 0.7, 1.0, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9, 2.2, 2.5, 2.8, 3.1, 3.4, 3.7, 4.0]'; 
yObstacles = [0.3, 0.4, 0.5, 0.6, 0.7, 1.0, 1.3, 1.4, 1.5, 1.6, 1.8, 1.9, 2.2, 2.5, 2.8, 3.1, 3.4, 3.7, 4.0]';

[transitionPoses] = PlanAndShowRRTPath(startPose, goalPose, xObstacles, yObstacles);

figure
scatter(transitionPoses(:,1),transitionPoses(:,2),[],'filled', ...
    'DisplayName','Transition Poses')
xlim([0 4.5])
ylim([0 4])