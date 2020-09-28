
% to be created from VehicleObservation
startPoses = struct('vehicle_1', []);
startPoses.vehicle_1.x = 2.3;
startPoses.vehicle_1.y = 1.5;
%actually rad
startPoses.vehicle_1.yaw = 225;

startPoses.vehicle_3 = [];
startPoses.vehicle_3.x = 0.2;
startPoses.vehicle_3.y = 3.5;
%actually rad
startPoses.vehicle_3.yaw = 180;

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

startPoses.vehicle_9 = [];
startPoses.vehicle_9.x = 0.7;
startPoses.vehicle_9.y = 2.5;
%actually rad
startPoses.vehicle_9.yaw = 270;

%TODO if input arg when calling script from LCC with arg 'comeHome'
goalPoses = setHomePoses(startPoses);
%TODO 
egoVehicle = 'vehicle_1';
speed = 1; % [m/s]
refPath = PlanAndShowRRTPath(startPoses, goalPoses, egoVehicle);
trajectory_points = pathToTrajectory(refPath, speed);

%TODO: what to do when goalPoses occupied by undesired vehicle?
%Let vehicle be removed by hand or program?
