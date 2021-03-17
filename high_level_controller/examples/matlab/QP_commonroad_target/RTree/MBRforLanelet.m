% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

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