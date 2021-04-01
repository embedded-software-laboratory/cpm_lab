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

% function which computes the points in global coordinates from local
% coordinate points
function globalPoints=local2global(progress,ref_points,currPos)
polyline = [currPos, ref_points];
delta_s = diff(polyline,1,2);
delta_s_distance = [0 , sqrt(delta_s(1,:).^2 + delta_s(2,:).^2)];
    
local_coord = cumsum(delta_s_distance);

if progress(end) > local_coord(end)
    progress
    local_coord
    disp('HLC planned beyond lanelet horizon, warning');
    progress =min(progress,local_coord(end));
end

global_x = interp1(local_coord,polyline(1,:),progress);
global_y = interp1(local_coord,polyline(2,:),progress);

globalPoints=[global_x; global_y];


end