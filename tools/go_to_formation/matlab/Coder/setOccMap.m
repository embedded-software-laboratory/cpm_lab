function [map] = setOccMap(vehicleList, vehiclePoses, egoVehicle)

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
corners = struct;

for nVehicles = 1:length(vehicleList)
 
    if vehicleList{nVehicles} == egoVehicle
        continue
    end
    
    corners.(vehicleList{nVehicles}).x(1,1) = vehiclePoses.(vehicleList{nVehicles}).x + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).x(2,1) = vehiclePoses.(vehicleList{nVehicles}).x + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).x(3,1) = vehiclePoses.(vehicleList{nVehicles}).x - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).x(4,1) = vehiclePoses.(vehicleList{nVehicles}).x - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    
    corners.(vehicleList{nVehicles}).y(1,1) = vehiclePoses.(vehicleList{nVehicles}).y + sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).y(2,1) = vehiclePoses.(vehicleList{nVehicles}).y + sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).y(3,1) = vehiclePoses.(vehicleList{nVehicles}).y - sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    corners.(vehicleList{nVehicles}).y(4,1) = vehiclePoses.(vehicleList{nVehicles}).y - sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    
    %calculate vehicle edges as rays along corners
    for i = 1:4
        if i < 4
        [~,midpoints] = raycast(map, [corners.(vehicleList{nVehicles}).x(i) corners.(vehicleList{nVehicles}).y(i)],...
                                             [corners.(vehicleList{nVehicles}).x(i+1) corners.(vehicleList{nVehicles}).y(i+1)]);
                                     
        else
        [~,midpoints] = raycast(map, [corners.(vehicleList{nVehicles}).x(4) corners.(vehicleList{nVehicles}).y(4)],...
                                             [corners.(vehicleList{nVehicles}).x(1) corners.(vehicleList{nVehicles}).y(1)]); 
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