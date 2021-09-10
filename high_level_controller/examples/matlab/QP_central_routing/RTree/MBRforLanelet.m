function [newRect, Polygon] =  MBRforLanelet(inputLaneletChildren)
%% function call to calculate the polygon and minimum bounding rectangle for a lanelet
%
%
numEntries=size(inputLaneletChildren,2);
PointsLB = zeros(2,2); % points of leftBound, at least 2 Points
PointsRB = zeros(2,2); % points of rightBound, at least 2 Points


lBFound = 0;
rBFound = 0;
for k = 1:numEntries
    if strcmp(inputLaneletChildren(k).Name, 'leftBound')
        leftBound = inputLaneletChildren(k).Children;
        lBFound = 1;
    end
    if strcmp(inputLaneletChildren(k).Name, 'rightBound')
        rightBound = inputLaneletChildren(k).Children;
        rBFound=1;
    end
    
    
end

if size(leftBound,2) ~= size(rightBound,2)
    error('num of leftBound not equal num of rightBound');
end

minX = Inf;
minY = Inf;
maxX = -Inf;
maxY =  -Inf;
counter = 1;
if lBFound && rBFound
    numEntry=size(leftBound,2);
    for k = 1:numEntry
        if strcmp(leftBound(k).Name,'point') && strcmp(rightBound(k).Name,'point')
            newPointX = str2double(leftBound(k).Children(1).Children.Data);
            newPointY = str2double(leftBound(k).Children(2).Children.Data);
            if minX > newPointX
                minX = newPointX;
            end
            if minY > newPointY
                minY = newPointY;
            end
            if maxX < newPointX
                maxX = newPointX;
            end
            if maxY <newPointY
                maxY = newPointY;
            end
            
            PointsLB(:,counter) = [newPointX; newPointY]; % dynamically increase size
            

            newPointX = str2double(rightBound(k).Children(1).Children.Data);
            newPointY = str2double(rightBound(k).Children(2).Children.Data);
            if minX > newPointX
                minX = newPointX;
            end
            if minY > newPointY
                minY = newPointY;
            end
            if maxX < newPointX
                maxX = newPointX;
            end
            if maxY <newPointY
                maxY = newPointY;
            end
            PointsRB(:,counter) = [newPointX; newPointY]; % dynamically increase size
            counter = counter +1; 
        end
            
    end
    
    lle = Point2D_RTree(minX,minY);
    rue = Point2D_RTree(maxX,maxY);
    
    newRect= Rectangle(lle,rue);
    Polygon = [PointsRB , flip(PointsLB,2)];
    
else
    if lBFound == 0
        error('leftBound not found');
    else
        error('rightBound not found');
    end
    
end


end