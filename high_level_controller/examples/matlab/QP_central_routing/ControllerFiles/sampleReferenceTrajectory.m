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


% Computes reference trajectory points for a piecewise linear curve. Makes uses of the
% reference speed on path points.

function [ RefData ] = sampleReferenceTrajectory(Hp, path_pos, path_speed,currPos,dt)

    % referenceTrajectory is non-equidistant array of centerline [[x1;y1],...,[xn;yn]] 
    % where [x1;y1] is the next sampling point in driving direction
    PosTrajectory = [currPos,path_pos];
    DeltaP = diff( PosTrajectory,1,2);
    DeltaS = [sqrt(DeltaP(1,:).^2 + DeltaP(2,:).^2)];
    s_coordinate = [0, cumsum(DeltaS)]; % local coordinate system
    maxDist = s_coordinate(end); % maximum allowed distance in horizon
    
    ref_speed = [path_speed(1) , path_speed];
     
    % just to be sure that there is enough trajectory points available for collision handling
    n_path =   Hp+5; % number of path points
    samplingPos = zeros(1,n_path); % reference trajectory : position
    samplingSpeed = zeros(1,n_path);
    
    samplingPos(1) = 0 + path_speed(1) * dt;
    thresholdIndex = find( s_coordinate > samplingPos(1));
    
    %the point has to be between SpeedIndex and SpeedIndex +1. 
    SpeedIndex = min (max (thresholdIndex(1)-1,1), n_path-1 ); 
    samplingSpeed(1) = min( ref_speed(SpeedIndex),ref_speed(SpeedIndex+1)) ;
    for k=2:n_path
        samplingPos(k) = samplingPos(k-1) + samplingSpeed(k-1) *dt;
        if maxDist <= samplingPos(k)
            % 0*sampling just for dimension
            samplingPos(k:end) = 0*samplingPos(k:end) + maxDist;
            samplingSpeed(k:end) = 0 * samplingSpeed(k:end) + ref_speed(end); 
            break;
        else
            SpeedIndex = find( s_coordinate > samplingPos(k));
            samplingSpeed(k) = ref_speed(SpeedIndex(1));
        end
        
    end
    
 
    
    % local coordinate for mpc
    Ref_pos_mpc = samplingPos(1:Hp); % pos
    Ref_pos_mpc = min(Ref_pos_mpc,maxDist) ;% we want to stop at maxDist

    Ref_speed_mpc =  interp1(s_coordinate,ref_speed,Ref_pos_mpc); % desired speed, y2  
    % local coordinate for collision handling
    Ref_Points_ch = samplingPos(1:n_path);
    Ref_Points_ch = min(Ref_Points_ch,maxDist); % we want to stop at maxDist
    
    Ref_x_global = interp1(s_coordinate,PosTrajectory(1,:),Ref_Points_ch);
    Ref_y_global = interp1(s_coordinate,PosTrajectory(2,:),Ref_Points_ch);
    
    Ref_points_global = [Ref_x_global;Ref_y_global]; % global coordinate
    
    
    RefData.TrajPointsGlobal =Ref_points_global ; % global coordinate , reference for collision handling
    
    RefData.ReferenceTrajectoryPoints = [Ref_pos_mpc;Ref_speed_mpc]; % Reference for mpc, local coordinate
    
end

