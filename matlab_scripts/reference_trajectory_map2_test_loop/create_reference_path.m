function create_reference_path
    
    path_data = load('../map_print/map_print2/cache.mat');
    

    segment_sequence = struct;
    i = 1;
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_F;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_C;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_G;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 2;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_D;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_E;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_B;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_B;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_E;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_D;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_G;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 2;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_C;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_F;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_F;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_C;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_G;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 2;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_D;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_E;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_B;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_B;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_E;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_D;
    segment_sequence(i).sx = 1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_G;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = -1;
    segment_sequence(i).offset = 2;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_C;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = -1;
    i = i + 1;
    
    
    segment_sequence(i).segment = path_data.segment_F;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    segment_sequence(i).segment = path_data.segment_A;
    segment_sequence(i).sx = -1;
    segment_sequence(i).sy = 1;
    segment_sequence(i).offset = 0;
    i = i + 1;
    
    
    % calculate segment trajectory transformations
    for i = 1:length(segment_sequence)
        
        x = segment_sequence(i).segment.x;
        y = segment_sequence(i).segment.y;
        sx = segment_sequence(i).sx;
        sy = segment_sequence(i).sy;
        w = path_data.lane_width;
        yaw = segment_sequence(i).segment.yaw;
        c = segment_sequence(i).offset * w * cos(yaw);
        s = segment_sequence(i).offset * w * sin(yaw);

        segment_sequence(i).trajectory_x = sx * (x-s);
        segment_sequence(i).trajectory_y = sy * (y+c);
    end
    
    
    % find connecting indices of consecutive segments
    for i = 1:length(segment_sequence)
        i2 = mod(i, length(segment_sequence)) + 1;
        
        dx = segment_sequence(i2).trajectory_x - segment_sequence(i).trajectory_x';
        dy = segment_sequence(i2).trajectory_y - segment_sequence(i).trajectory_y';
        
        dist_sq = dx.^2 + dy.^2;
        [min_dist_sq, min_dist_idx] = min(dist_sq(:));
        
        [min_dist_idx_A, min_dist_idx_B] = ind2sub(size(dx), min_dist_idx);
        
        assert(min_dist_sq < 1e-4);
        
        segment_sequence(i).end_idx = min_dist_idx_B;
        segment_sequence(i2).start_idx = min_dist_idx_A;
    end
    
    % trim segments, so that the ends connect
    for i = 1:length(segment_sequence)
        start_idx = segment_sequence(i).start_idx;
        end_idx = segment_sequence(i).end_idx;
        assert(abs(start_idx - end_idx) > 10);
        
        if start_idx < end_idx
            slice = start_idx:end_idx;
        else
            slice = start_idx:-1:end_idx;
        end
        
        segment_sequence(i).trajectory_trim_x = segment_sequence(i).trajectory_x(slice);
        segment_sequence(i).trajectory_trim_y = segment_sequence(i).trajectory_y(slice);
    end
    
    % combine segments
    trajectory_x = [];
    trajectory_y = [];
    for i = 1:length(segment_sequence)
        trajectory_x = [trajectory_x; segment_sequence(i).trajectory_trim_x];
        trajectory_y = [trajectory_y; segment_sequence(i).trajectory_trim_y];
    end
    
    
    repeat_filter = ((diff(trajectory_x).^2 + diff(trajectory_y).^2) > 1e-12);
    trajectory_x = trajectory_x(repeat_filter);
    trajectory_y = trajectory_y(repeat_filter);
    
    
    % equidistant re-interpolation
    ds = sqrt(diff(trajectory_x).^2 + diff(trajectory_y).^2);
    s = [0; cumsum(ds)];
    s2 = linspace(min(s), max(s), round(max(s)/0.01));
    s2 = s2(2:end);
    
    trajectory_x = interp1(s,trajectory_x,s2,'linear');
    trajectory_y = interp1(s,trajectory_y,s2,'linear');
    
    trajectory_x = trajectory_x + 2.25;
    trajectory_y = trajectory_y + 2.0;
    
    dlmwrite('reference_path.csv', [trajectory_x', trajectory_y'],'precision',10);
    
    
%     for j = 1:length(segment_sequence)
%         clf
%         hold on
%         for i = max((j-5),1):j            
%             plot(  ...
%                 segment_sequence(i).trajectory_trim_x,  ...
%                 segment_sequence(i).trajectory_trim_y   ...
%             );
%         end
% 
%         axis equal
%         xlim([-3;3])
%         ylim([-3;3])
%         drawnow
%         pause(0.2)
%     end
    
end


