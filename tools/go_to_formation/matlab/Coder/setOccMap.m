function [map] = setOccMap(vehiclePoses, egoVehicleId)

mapX = 4.5;
mapY = 4.0;
resolution = 100;

map = occupancyMap(mapX, mapY, resolution);
mat = zeros(mapY * resolution, mapX * resolution);
setOccupancy(map, [0 0], mat)

%% Calculate corners and edges of vehicles from pose
vehicleLength = 0.2200;
vehicleWidth= 0.1070;
vehicleHalfLength = vehicleLength/2;
vehicleHalfWidth = vehicleWidth/2;

% calculate corners of vehicle
occval = 1;
corners = struct('corners_x', zeros(4,1), 'corners_y', zeros(4,1));
isVehicleDeployed = true;
maxNoVehicles = 20;
midpoints = zeros(1,20);

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
    
    corners(nVehicles).corners_x(1) = vehiclePoses(nVehicles).pose.x + cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_x(2) = vehiclePoses(nVehicles).pose.x + cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_x(3) = vehiclePoses(nVehicles).pose.x - cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_x(4) = vehiclePoses(nVehicles).pose.x - cosd(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + sind(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    
    corners(nVehicles).corners_y(1) = vehiclePoses(nVehicles).pose.y + sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_y(2) = vehiclePoses(nVehicles).pose.y + sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_y(3) = vehiclePoses(nVehicles).pose.y - sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength + cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    corners(nVehicles).corners_y(4) = vehiclePoses(nVehicles).pose.y - sind(vehiclePoses(nVehicles).pose.yaw) * vehicleHalfLength - cosd(vehiclePoses(nVehicles).pose.yaw)* vehicleHalfWidth;
    
    %calculate vehicle edges as rays along corners
    for i = 1:4
        if i < 4
        [~,midpoints] = raycast(map, [corners(nVehicles).corners_x(i) corners(nVehicles).corners_y(i)],...
                                     [corners(nVehicles).corners_x(i+1) corners(nVehicles).corners_y(i+1)]);
                                     
        else
        [~,midpoints] = raycast(map, [corners(nVehicles).corners_x(4) corners(nVehicles).corners_y(4)],...
                                     [corners(nVehicles).corners_x(1) corners(nVehicles).corners_y(1)]); 
        end
        setOccupancy(map,midpoints,occval,"grid");
    end
    
end  
    

% set map limits % todo: set corners of map
[~,midpoints] = raycast(map, [0 0], [0 mapY]);
setOccupancy(map,midpoints,occval,"grid")

[~,midpoints] = raycast(map, [0 mapY], [mapX mapY]);
setOccupancy(map,midpoints,occval,"grid")

[~,midpoints] = raycast(map, [mapX mapY], [mapX 0]);
setOccupancy(map,midpoints,occval,"grid")

[~,midpoints] = raycast(map, [mapX 0], [0 0]);
setOccupancy(map,midpoints,occval,"grid")

inflate(map, 0.05)
end