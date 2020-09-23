
% to be created from VehicleObservation
startPoses = struct('vehicle_1', []);
startPoses.vehicle_1.x = 2.3;
startPoses.vehicle_1.y = 1.5;
%actually rad
startPoses.vehicle_1.yaw = 225;


startPoses.vehicle_4 = [];
startPoses.vehicle_4.x = 1.3;
startPoses.vehicle_4.y = 2.5;
%actually rad
startPoses.vehicle_4.yaw = 125;

startPoses.vehicle_7 = [];
startPoses.vehicle_7.x = 2.3;
startPoses.vehicle_7.y = 2.5;
%actually rad
startPoses.vehicle_7.yaw = 25;

goalPoses = setHomePoses(startPoses);
[transitionPoses] = PlanAndShowRRTPath(startPoses, goalPoses);

% figure
% scatter(transitionPoses(:,1),transitionPoses(:,2),[],'filled', ...
%     'DisplayName','Transition Poses')
% xlim([0 4.5])
% ylim([0 4])