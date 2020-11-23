function [xMapLimits, yMapLimits] = SetMapLimits(mapX, mapY, cellSize) %#codegen

    %number of grid cells on each axis
    xCellNum = mapX/cellSize;
    yCellNum = mapY/cellSize;
    %values of grid cells on axis
    xValues = linspace(0, mapX, xCellNum);
    yValues = linspace(0, mapY, yCellNum);


    %Eastern Limits
    xEasternLimits = zeros(yCellNum, 1);
    yEasternLimits = yValues';

    %Southern Limits
    xSouthernLimits = xValues';
    ySouthernLimits = zeros(xCellNum, 1);

    %Western Limits
    xWesternLimits = ones(yCellNum, 1)*mapX;
    yWesternLimits = yValues';

    %Northern Limits
    xNorthernLimits = xValues';
    yNorthernLimits = ones(xCellNum, 1)*mapY;


    xMapLimits = vertcat(xEasternLimits, xSouthernLimits, xWesternLimits, xNorthernLimits);
    yMapLimits = vertcat(yEasternLimits, ySouthernLimits, yWesternLimits, yNorthernLimits);

end