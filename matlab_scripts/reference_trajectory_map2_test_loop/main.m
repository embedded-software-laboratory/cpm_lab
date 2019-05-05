function main
    
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
    
    
    
    
    
    for j = 1:length(segment_sequence)
        clf
        hold on
        for i = max((j-5),1):j
            

            x = segment_sequence(i).segment.x;
            y = segment_sequence(i).segment.y;
            sx = segment_sequence(i).sx;
            sy = segment_sequence(i).sy;
            w = path_data.lane_width;
            yaw = segment_sequence(i).segment.yaw;
            c = segment_sequence(i).offset * w * cos(yaw);
            s = segment_sequence(i).offset * w * sin(yaw);
            plot(  ...
                sx * (x-s),  ...
                sy * (y+c)   ...
            );


        end

        axis equal
        xlim([-3;3])
        ylim([-3;3])
        drawnow
        pause(0.2)
    end
    
end


