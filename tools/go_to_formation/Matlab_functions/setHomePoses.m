function [goalPoses] = setHomePoses(startPoses)

mapY = 4.0;

% vehicle dimensions [m]
vehicleLength = 0.2200;
vehicleWidth= 0.1070;
vehicleHalfLength = vehicleLength/2;
vehicleHalfWidth = vehicleWidth/2;

%arrangement of vehicles
maxNoVehicles = 20;
vehicleRows = 3;
vehicleColumns = ceil(maxNoVehicles/vehicleRows);
clearance = 0.1; % safety distance vehicle to vehicle and vehicle to map edge [m]

goalPoses = startPoses;
homePoses = zeros(maxNoVehicles, 3);
homePoses(:, 3) = 90; %yaw [degree] - all vehicles should align with y axis

%TODO: set to fixed to decrease runtime?
for nHomePoses = 1:maxNoVehicles
    
    if mod(nHomePoses, vehicleColumns) == 0
        column = vehicleColumns; % vehicle count in y (=rows)
        row = floor(nHomePoses/vehicleColumns); % vehicle count in x (=columns)
    else 
        column = mod(nHomePoses, vehicleColumns);
        row = 1 + floor(nHomePoses/vehicleColumns); % vehicle count in x (=columns)
    end
    homePoses(nHomePoses, 1) = vehicleHalfWidth + column * clearance + (column-1) * vehicleWidth ;
    homePoses(nHomePoses, 2) = mapY - vehicleHalfLength - row * clearance - (row-1) * vehicleLength;
end   


vehicleList = fields(goalPoses);
for nVehicles = 1:length(vehicleList)
    
    goalPoses.(vehicleList{nVehicles}).x = homePoses(nVehicles, 1);
    goalPoses.(vehicleList{nVehicles}).y = homePoses(nVehicles, 2);
    goalPoses.(vehicleList{nVehicles}).yaw = homePoses(nVehicles, 3);
    
end
