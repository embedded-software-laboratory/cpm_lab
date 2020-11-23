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