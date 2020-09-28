function [map] = setOccMap(vehiclePoses, egoVehicle)

mapX = 4.5;
mapY = 4.0;
resolution = 100;

map = occupancyMap(mapX, mapY, resolution);
mat = zeros(mapY * resolution, mapX * resolution);
setOccupancy(map, [0 0], mat)

%% 
vehicleLength = 0.2200;
vehicleWidth= 0.1070;
vehicleHalfLength = vehicleLength/2;
vehicleHalfWidth = vehicleWidth/2;

% calculate corners of vehicle
vehicleList = fields(vehiclePoses);
occval = 1;


for nVehicles = 1:length(vehicleList)
    
    if vehicleList{nVehicles} == egoVehicle
        continue
    end
    
    vehiclePoses.(vehicleList{nVehicles}).corners.x(1,1) = vehiclePoses.(vehicleList{nVehicles}).x + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.x(2,1) = vehiclePoses.(vehicleList{nVehicles}).x + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.x(3,1) = vehiclePoses.(vehicleList{nVehicles}).x - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.x(4,1) = vehiclePoses.(vehicleList{nVehicles}).x - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + sind(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    
    vehiclePoses.(vehicleList{nVehicles}).corners.y(1,1) = vehiclePoses.(vehicleList{nVehicles}).y + sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.y(2,1) = vehiclePoses.(vehicleList{nVehicles}).y + sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.y(3,1) = vehiclePoses.(vehicleList{nVehicles}).y - sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength + cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    vehiclePoses.(vehicleList{nVehicles}).corners.y(4,1) = vehiclePoses.(vehicleList{nVehicles}).y - sind(vehiclePoses.(vehicleList{nVehicles}).yaw) * vehicleHalfLength - cosd(vehiclePoses.(vehicleList{nVehicles}).yaw)* vehicleHalfWidth;
    
    %calculate rays along corners of vehicles
    for i = 1:4
        if i < 4
        [endpoints,midpoints] = raycast(map, [vehiclePoses.(vehicleList{nVehicles}).corners.x(i) vehiclePoses.(vehicleList{nVehicles}).corners.y(i)],...
                                             [vehiclePoses.(vehicleList{nVehicles}).corners.x(i+1) vehiclePoses.(vehicleList{nVehicles}).corners.y(i+1)]);
                                     
        else
        [endpoints,midpoints] = raycast(map, [vehiclePoses.(vehicleList{nVehicles}).corners.x(4) vehiclePoses.(vehicleList{nVehicles}).corners.y(4)],...
                                             [vehiclePoses.(vehicleList{nVehicles}).corners.x(1) vehiclePoses.(vehicleList{nVehicles}).corners.y(1)]); 
        end
        setOccupancy(map,midpoints,occval,"grid");
    end
    
end  
    

% set map limits % todo: set corners of map
[endpoints,midpoints] = raycast(map, [0 0], [0 mapY]);
setOccupancy(map,midpoints,occval,"grid")

[endpoints,midpoints] = raycast(map, [0 mapY], [mapX mapY]);
setOccupancy(map,midpoints,occval,"grid")

[endpoints,midpoints] = raycast(map, [mapX mapY], [mapX 0]);
setOccupancy(map,midpoints,occval,"grid")

[endpoints,midpoints] = raycast(map, [mapX 0], [0 0]);
setOccupancy(map,midpoints,occval,"grid")
