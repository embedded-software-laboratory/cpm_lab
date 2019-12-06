function [trajectory_row] = get_trajectory(pos)
    trajectory_px = [1, 0,-1, 0];
    trajectory_py = [0, 1, 0,-1];
    trajectory_vx = [0,-1, 0, 1];
    trajectory_vy = [1, 0,-1, 0];
    %Get size
    [m,n] = size(trajectory);

    %modulo nutzen
    if pos >= 1
        pos = mod(pos-1, m)+1;
        trajectory_row = trajectory(pos,:);
    else
        error('Wrong index for pos!');
    end
end