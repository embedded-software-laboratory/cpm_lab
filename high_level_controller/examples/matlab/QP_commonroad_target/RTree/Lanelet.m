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

classdef Lanelet < handle
%       Constructor
%             obj=MyLanelet(new_lanelet)  
%       Map-pos-Matching
%             [next_edge_index, lambda] = find_next_edge(obj,pos):
%                    lambda measures the remaining length of the lane
%             [next_id, hasSucc] = choose_random_successor(obj)
%       Drawing Function 
%             drawLanelet(obj)
%             drawCenterline(obj)

            
    properties (Access = public)
        
        numPred=int8(0);
        numSucc=int8(0);
        PredList=int32([]); % id list as integer array
        SuccList=int32([]); % id list as integer arary
        numEdges=int16(0);
        leftBound; %[x1;y1 ]
        rightBound; % [x1;y1 ]
        centerline; % [x1;y1 ]
        centerlineDist; % distance between centerline points
        speedProfile; % [vel1,...veln] in [m/s]
        tangentVector; %[x1;y1 ]
        MBR; % minimum bounding Rectangle
        polygon; 
        id;
        speedLimit=9999; % [m/s]
        hasSpeedLimit=int8(1); % flag currently always true as we use dummy speedLimit of 9999;
        adjLeft_id=int32([]);
        adjRight_id=int32([]);
        adjLeft_dir=int8([]); % same direction is 1,  opposite direction is 0
        adjRight_dir=int8([]); % same direction is 1,  opposite direction is 0
    end
    
    methods
        
        function obj=Lanelet(new_lanelet)
            % reads parsed commonroad xml file into a lanelet object
             %% check if id exists
            newId_found = int8(0);
            for j = 1: size( new_lanelet.Attributes,2)
                if strcmp(new_lanelet.Attributes.Name,'id')
                    new_id =int32(str2double(new_lanelet.Attributes.Value));
                    newId_found = 1;
                    break;
                end
            end
            if ~newId_found
                error(['lanelet has no id']);
            end
            
            obj.id=new_id;
            
            numEntries=size(new_lanelet.Children,2);
            
            %% extract information speedLimit,adjacent lanes, pred, succ
            eps = 1e-12; %threshold for zero
            
            for k = 1:numEntries
                if strcmp(new_lanelet.Children(k).Name, 'speedLimit')
                    obj.speedLimit = str2double(new_lanelet.Children(k).Children.Data)/3.6; % convert from km/h to m/s
                    obj.hasSpeedLimit = 1;
                end
                if strcmp(new_lanelet.Children(k).Name, 'adjacentRight')
                    
                    if ~ (strcmp(new_lanelet.Children(k).Attributes(2).Name, 'ref') && strcmp(new_lanelet.Children(k).Attributes(1).Name,'drivingDir' ))
                        error(['adjacentRight not in format [DrivingDir;ref] in lanelet ', num2str(k) ]);
                    end
                    
                   
                    obj.adjRight_id = [obj.adjRight_id, new_id];
                    
                    dir =  new_lanelet.Children(k).Attributes(1).Value;
                    
                    if strcmp(dir,'same')
                        obj.adjRight_dir = [obj.adjRight_dir, 1];
                    else
                        obj.adjRight_dir= [obj.adjRight_dir,0];
                    end
                end
                if strcmp(new_lanelet.Children(k).Name, 'adjacentLeft')
                    if ~ (strcmp(new_lanelet.Children(k).Attributes(2).Name, 'ref') && strcmp(new_lanelet.Children(k).Attributes(1).Name,'drivingDir' ))
                        error(['adjacentLeft not in format [DrivingDir;ref] in lanelet ', num2str(k) ]);
                    end
                    
                    new_id = int8( str2double( new_lanelet.Children(k).Attributes(1).Value));
                    obj.adjLeft_id = [obj.adjLeft_id, new_id];
                    dir =  new_lanelet.Children(k).Attributes(1).Value(1);
                    if strcmp(dir,'same')
                        obj.adjLeft_dir = [obj.adjLeft_dir, 1];
                    else
                        obj.adjLeft_dir= [obj.adjLeft_dir,0];
                    end
                    1;
                end
                if strcmp(new_lanelet.Children(k).Name, 'predecessor')
                    pred_id = str2double(new_lanelet.Children(k).Attributes.Value);
                    obj.PredList = [obj.PredList, pred_id];
                    obj.numPred=obj.numPred+1;
                end
                if strcmp(new_lanelet.Children(k).Name, 'successor')
                    succ_id = str2double(new_lanelet.Children(k).Attributes.Value);
                    obj.SuccList = [obj.SuccList, succ_id];
                    obj.numSucc=obj.numSucc+1;
                end
            end
            
            %% import left and rightbound, calculate polygon, centerline and minimum bounding rectangle
            
            PointsLB = zeros(2,2); % points of leftBound, at least 2 Points
            PointsRB = zeros(2,2); % points of rightBound, at least 2 Points
            
            
            lBFound = 0;
            rBFound = 0;
            for k = 1:numEntries
                if strcmp(new_lanelet.Children(k).Name, 'leftBound')
                    leftBound = new_lanelet.Children(k).Children;
                    lBFound = 1;
                end
                if strcmp(new_lanelet.Children(k).Name, 'rightBound')
                    rightBound = new_lanelet.Children(k).Children;
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
                
                obj.MBR= Rectangle(lle,rue);
                
                centerline = 0.5 * (PointsLB+PointsRB);
                xyDist = centerline(:,2:end)-centerline(:,1:end-1);
                absDist = sqrt(xyDist(1,:).^2 + xyDist(2,:).^2);
               
                numPoints = size(PointsLB,2);
                % eliminate points with zero distance
                counter = 0;
                if numPoints > 3 % nothing to do if only starting and end point are given
                     k = 1; % cannot delete starting point,therefore special case 
                      if absDist(k) < eps
                          absDist(k)=[];
                          PointsLB(:,k+1) = [];
                          PointsRB(:,k+1) = [];
                          centerline(:,k+1) = [];
                          counter = counter +1;
                      end
                    
                     for k=2:numPoints-1
                      if absDist(k-counter) < eps
                          absDist(k-counter)=[];
                          PointsLB(:,k-counter) = [];
                          PointsRB(:,k-counter) = [];
                          centerline(:,k+1) = [];
                          counter = counter +1;
                      end
                    end
                end
                
                obj.centerlineDist = absDist;      
                obj.leftBound = PointsLB;
                obj.rightBound = PointsRB;
                obj.centerline = centerline;
                obj.speedProfile = obj.speedLimit + 0 * centerline(1,:); % set to constant, speedlimit
                obj.polygon = [PointsRB , flip(PointsLB,2)];
                obj.numEdges = size(centerline,2);
            else
                if lBFound == 0
                    error('leftBound not found');
                else
                    error('rightBound not found');
                end
                
            end
            
            %% calculate tangent vectors 
            % with centerline. in case lanelet width changes
            
            centerline =obj.centerline;
            numTangents = obj.numEdges;
            diffBound = (obj.leftBound - obj.rightBound);
            tangentVector = [-diffBound(2,:); diffBound(1,:)];
            % scale tangentVector to unit length
            tangentVector(:,1) = tangentVector(:,1) / length(tangentVector(:,1));
            
            % scaling such that the projection on deltaP is the same
            for k =2:numTangents
                deltaP = centerline(:,k)-centerline(:,k-1);
                denom = (deltaP' * tangentVector(:,k ) );
                numerator = deltaP'*tangentVector(:,k-1 ) ;
                  scal =  numerator /  denom;

                tangentVector(:,k ) =scal * tangentVector(:,k);
                
                % assertion
                deltaT = (tangentVector(:,k)-tangentVector(:,k-1));
                
                assert(deltaP'*deltaT < eps); % == 0 (almost zero, orthogonal)
                   
            end
            
            obj.tangentVector = tangentVector;
            
           
            
            
        end
            
        
        %% find first edge
        % function calculating the next centerline edge in driving
        % direction for a given point pos=[x,y]; 
        % Important: Assumes pos is in lanelet
        % returns: index of next edge, and 0<lambda<1 progress in this zone
        function [next_edge_index, lambda] = find_next_edge(obj,pos)
            if size(pos,1) == 1
               pos  = pos'; 
            end
            
            numZones = obj.numEdges-1;
            next_edge_index = int16(0);
            lambda = 0;
            zoneFound = int8(0);
            for k = 1:numZones
                newPolygon = [obj.leftBound(:,k) , obj.leftBound(:,k+1), obj.rightBound(:,k+1),obj.rightBound(:,k)];
                if inpolygon(pos(1),pos(2),newPolygon(1,:),newPolygon(2,:))
                    zoneFound=1;
                    next_edge_index = k+1;
                    
                    p2 = obj.centerline(:,k+1);
                    p1 = obj.centerline(:,k);
                    t2 = obj.tangentVector(:,k+1);
                    t1 = obj.tangentVector(:,k);
                    z=pos;
                    
                     numerator = (p2-z)'*t2;
                     denominator = (p2-p1)'*t2 + (z-p2)'*(t1-t2);
                    lambda = numerator/denominator;
                    
                   break; 
                end
            end
            if zoneFound ==0
                 error(['did not match position, probably wrong lanelet id: ',num2str(obj.id)]);
            end
        end
        
%          %% find first edge
%         % function calculating the next centerline edge in driving
%         % direction for a given point pos=[x,y]; 
%         % Important: does not assume pos is in lanelet, simply choses
%         % closest segment
%         % returns: index of next edge, and lambda progress in this zone
%      
%         function [next_edge_index, lambda] = find_next_edge2(obj,pos)
%                         
%             % calculate closest edge to pos
%              dist = (obj.centerline(1,:)-pos(1)).^2 +(obj.centerline(2,:)-pos(2)).^2;
%             [~,I] = min(dist);
%             
%             % compare neighbours of closest edge to choose segment            
%             if I == 1
%                 next_edge_index= 2;
%             else
%                 if I == obj.numEdges
%                     next_edge_index=obj.numEdges;
%                 else
%                     dist_plus = (obj.centerline(1,I+1)-pos(1)).^2 + (obj.centerline(2,I+1)-pos(2)).^2;
%                     dist_minus = (obj.centerline(1,I-1)-pos(1)).^2 + (obj.centerline(2,I-1)-pos(2)).^2;
%                     
%                     if dist_plus <= dist_minus
%                         next_edge_index = I+1;
%                     else
%                         next_edge_index= I;
%                     end
%                 end
%             end
%             
%             
%             
%             % project into segment
%             
%             % get progress 
%                     k = next_edge_index -1;
%                     
%                     p2 = obj.centerline(:,k+1);
%                     p1 = obj.centerline(:,k);
%                     t2 = obj.tangentVector(:,k+1);
%                     t1 = obj.tangentVector(:,k);
%                     z=pos;
%                     
%                      numerator = (p2-z)'*t2;
%                      denominator = (p2-p1)'*t2 + (z-p2)'*(t1-t2);
%                     lambda = numerator/denominator;
%                     
%          
%                  
%             
%         end
        
%         
        function [proj_pos] = projectionOnCenterline(obj,pos)
            [next_edge_index, lambda] = obj.find_next_edge(pos);
            
            proj_pos = obj.centerline(:,next_edge_index-1) * lambda + obj.centerline(:,next_edge_index) * (1-lambda);
            
        end
        
        function [next_id, hasSucc] = choose_random_successor(obj)
            if obj.numSucc ==0
                next_id = obj.id;
                hasSucc = 0;
            else
               next_id = obj.SuccList(randi(obj.numSucc));
                hasSucc = 1;
            end
            

        end
        %% draw Lanelets in Matlab
        function drawLanelet(obj)
            plot(obj.leftBound(1,:),obj.leftBound(2,:),'LineStyle','-','Marker','.','Color','k' );
            plot(obj.rightBound(1,:),obj.rightBound(2,:),'LineStyle','-','Marker','.','Color','k');
             plot(obj.leftBound(1,1),obj.leftBound(2,1),'Marker','d','Color','k' );
            plot(obj.rightBound(1,1),obj.rightBound(2,1),'Marker','d','Color','k');
        end
        
        function drawCenterline(obj)
            plot(obj.centerline(1,:),obj.centerline(2,:),'LineStyle','--','Marker','.','Color','b');
        end
        
        %% get
        function poly= get_polygon(obj)
            poly = obj.polygon;
        end
        function rect= get_MBR(obj)
            rect = obj.MBR;
        end
        function id = get_id(obj)
            id = obj.id;
        end
        function line = get_centerline(obj)
            line = obj.centerline;
        end
        function dist = get_centerlineDist(obj)
            dist = obj.centerlineDist;
        end
        function line = get_leftBound(obj)
            line = obj.leftBound;
        end
        function line = get_rightBound(obj)
            line = obj.rightBound;
        end
        function vec =get_tangentVector(obj)
            vec = obj.tangentVector;
        end
 
        function num = get_numSucc(obj)
            num=obj.numSucc;
        end
        function num = get_numPred(obj)
            num=obj.numPred;
        end
        function list = get_SuccList(obj)
            list =obj.SuccList;
        end
        function list = get_PredList(obj)
            list = obj.PredList;
        end
        
        function id = get_adjLeft_id(obj)
            id = obj.adjLeft_id;
        end
        
        function id = get_adjRight_id(obj)
            id = obj.adjRight_id;
        end
        
        function dir = get_adjLeft_dir(obj)
            dir = obj.adjLeft_dir;
        end
        
        function dir = get_adjRight_dir(obj)
            dir = obj.adjRight_dir;
        end
        
        function limit = get_speedlimit(obj)
            limit = obj.speedLimit;
        end
        function speedProfile = get_speedProfile(obj)
            speedProfile = obj.speedProfile;
        end
        %% set (no reason to set anything)
        
        
    end
    
    
    
    
end