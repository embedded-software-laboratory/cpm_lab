map = binaryOccupancyMap(4.5,4, 100);


x = [1; 2; 3; 4];
y = [1; 2; 3; 4];

setOccupancy(map, [x y], ones(4,1))
inflate(map, 0.05)
figure
show(map)