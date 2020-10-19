function[map] = createOccMap() 

mapX = 4.5;
mapY = 4.0;
resolution = 100;

map = occupancyMap(mapX, mapY, resolution);
mat = zeros(mapY * resolution, mapX * resolution);
setOccupancy(map, [0 0], mat)

end