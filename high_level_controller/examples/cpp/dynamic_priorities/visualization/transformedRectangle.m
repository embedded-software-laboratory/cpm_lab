function [ polygon_XY ] = transformedRectangle( x,y,angle, Length, Width )
% TRANSFORMED_RECTANGLE Creates a rectangle with the given position and orientation.

    unitSquare = [ 0 0 0 1; 1 0 0 1; 1 1 0 1; 0 1 0 1]';
    
    % Read this bottom-up
    polygon_XY =  makehgtform('translate',[x y 0]) ...
                * makehgtform('zrotate',angle) ...
                * makehgtform('scale',[Length Width 1]) ...
                * makehgtform('translate',[-.5 -.5 0]) ...
                * unitSquare;
            
    polygon_XY = polygon_XY(1:2,:);
end
