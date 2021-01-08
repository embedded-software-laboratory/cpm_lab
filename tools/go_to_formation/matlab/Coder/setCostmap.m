function [costmap] = setCostmap(vehiclePoses, egoVehicleId)
    %% Initialize costmap
    mapX = 4.5;
    mapY = 4.0;
    cellSize = 0.01;
    inflationRadius = 0.07;

    centerPlacements = [0.25, 0.5, 0.75]; 
    vehLength = 0.22;
    vehicleDims = vehicleDimensions(vehLength,0.107, 0.07,...
        'FrontOverhang', 0.04,...
        'RearOverhang',0.03);
    ccConfig = inflationCollisionChecker(vehicleDims, ...
        'CenterPlacements',centerPlacements,'InflationRadius',inflationRadius);

    costmap = vehicleCostmap(mapX, mapY, 0, 'CellSize', cellSize,'CollisionChecker', ccConfig);

    %% Calculate corners of vehicles from pose and set to costmap
    
    vehicleHalfLength = vehicleDims.Length/2;
    vehicleHalfWidth = vehicleDims.Width/2;
    costval = 1;
    isVehicleDeployed = true;
    maxNoVehicles = 20;
    corners = zeros(4,2);

    for nVehicles = 1:maxNoVehicles

        % All vehicles not actually deployed are ignored.
        % Passing of all theoretically deployable vehicles necessary for
        % automatic code generation from matlab.
        if vehiclePoses(nVehicles).pose.x == 0 &&...
           vehiclePoses(nVehicles).pose.y == 0 &&...
           vehiclePoses(nVehicles).pose.yaw == 0

           isVehicleDeployed = false;
        end

        if vehiclePoses(nVehicles).vehicle_id == egoVehicleId || ~isVehicleDeployed
            continue
        end

        corners(1,1) = vehiclePoses(nVehicles).pose.x + cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(2,1) = vehiclePoses(nVehicles).pose.x + cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(3,1) = vehiclePoses(nVehicles).pose.x - cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(4,1) = vehiclePoses(nVehicles).pose.x - cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;

        corners(1,2) = vehiclePoses(nVehicles).pose.y + sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(2,2) = vehiclePoses(nVehicles).pose.y + sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(3,2) = vehiclePoses(nVehicles).pose.y - sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        corners(4,2) = vehiclePoses(nVehicles).pose.y - sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
        
        setCosts(costmap, [corners(:,1) corners(:,2)], costval)
        setCosts(costmap, [vehiclePoses(nVehicles).pose.x vehiclePoses(nVehicles).pose.y], costval)

    end  
    
    % Set map limits to costmap
    [xMapLimits, yMapLimits] = SetMapLimits(mapX, mapY, cellSize);
    setCosts(costmap, [xMapLimits, yMapLimits], costval)

end