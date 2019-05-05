function collision_map = load_collision_map
    
    reference_path = dlmread('reference_path.csv');
    path_x = reference_path(:,1);
    path_y = reference_path(:,2);
    
    [PX1, PX2] = meshgrid(path_x, path_x);
    [PY1, PY2] = meshgrid(path_y, path_y);
    
    Dist_sq = (PX2 - PX1).^2 + (PY2 - PY1).^2;
    collision_map = exp(-Dist_sq / 0.1);
    
    %imagesc(flipud(collision_map))
end

