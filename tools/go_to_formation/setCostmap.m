function [costmap] = setCostmap(vehiclePoses, iEgoVeh)
    % vehiclePoses: 3-by-nVeh double
    % iEgoVeh: 1-by-1 uint, index of vehicle to plan trajectory
    %% Initialize costmap
    mapX = 4.5;
    mapY = 4.0;
    cellSize = 0.01;
    inflationRadius = 0.07;

    centerPlacements = [0.25, 0.5, 0.75]; 
    vehLength = 0.22;
    vehicleDims = vehicleDimensions( ...
        vehLength,0.107, 0.07,...
        'FrontOverhang', 0.04,...
        'RearOverhang',0.03 ...
    );
    ccConfig = inflationCollisionChecker( ...
        vehicleDims, ...
        'CenterPlacements',centerPlacements, ...
        'InflationRadius',inflationRadius ...
    );

    costmap = vehicleCostmap(mapX, mapY, 0, 'CellSize', cellSize,'CollisionChecker', ccConfig);

    %% Calculate corners of vehicles from pose and set to costmap
    vehicleHalfLength = vehicleDims.Length/2;
    vehicleHalfWidth = vehicleDims.Width/2;
    costval = 1;
    nVeh = size(vehiclePoses,2);
    corners = zeros(4,2);

    for iVeh = 1:nVeh
        if (iVeh == iEgoVeh)
            continue
        end

        corners(1,1) = vehiclePoses(1,iVeh) + cosd(vehiclePoses(3,iVeh)) * vehicleHalfLength + sind(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(2,1) = vehiclePoses(1,iVeh) + cosd(vehiclePoses(3,iVeh)) * vehicleHalfLength - sind(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(3,1) = vehiclePoses(1,iVeh) - cosd(vehiclePoses(3,iVeh)) * vehicleHalfLength - sind(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(4,1) = vehiclePoses(1,iVeh) - cosd(vehiclePoses(3,iVeh)) * vehicleHalfLength + sind(vehiclePoses(3,iVeh))* vehicleHalfWidth;

        corners(1,2) = vehiclePoses(2,iVeh) + sind(vehiclePoses(3,iVeh)) * vehicleHalfLength - cosd(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(2,2) = vehiclePoses(2,iVeh) + sind(vehiclePoses(3,iVeh)) * vehicleHalfLength + cosd(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(3,2) = vehiclePoses(2,iVeh) - sind(vehiclePoses(3,iVeh)) * vehicleHalfLength + cosd(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        corners(4,2) = vehiclePoses(2,iVeh) - sind(vehiclePoses(3,iVeh)) * vehicleHalfLength - cosd(vehiclePoses(3,iVeh))* vehicleHalfWidth;
        
        setCosts(costmap, [corners(:,1) corners(:,2)], costval)
        setCosts(costmap, [vehiclePoses(1,iVeh) vehiclePoses(2,iVeh)], costval)

    end  
    
    %% Set map limits to costmap
    % Number of grid cells on each axis
    xCellNum = mapX/cellSize;
    yCellNum = mapY/cellSize;
    
    % Values of grid cells on axis
    xValues = linspace(0, mapX, xCellNum);
    yValues = linspace(0, mapY, yCellNum);

    % Eastern Limits
    xEasternLimits = zeros(yCellNum, 1);
    yEasternLimits = yValues';

    % Southern Limits
    xSouthernLimits = xValues';
    ySouthernLimits = zeros(xCellNum, 1);

    % Western Limits
    xWesternLimits = ones(yCellNum, 1)*mapX;
    yWesternLimits = yValues';

    % Northern Limits
    xNorthernLimits = xValues';
    yNorthernLimits = ones(xCellNum, 1)*mapY;
    
    xMapLimits = [xEasternLimits; xSouthernLimits; xWesternLimits; xNorthernLimits];
    yMapLimits = [yEasternLimits; ySouthernLimits; yWesternLimits; yNorthernLimits];

    setCosts(costmap, [xMapLimits, yMapLimits], costval)

end