function main
    
    %% The map is made from 10*4 = 10 segments.
    %% The 10 segments are labeled A-K in "map-sketch.jpg".
    %% The map has mirror symmetry around the vertical 
    %% and horizontal axis, centered on the intersection, 
    %% giving 4x the segments.
    
    
    if isfile('cache.mat')
        load('cache.mat')
    else
        full_height = 2;
        full_width = 2.25;
        margin = 0.03;
        height = full_height - margin;
        width = full_width - margin;
        lane_width = 0.15;

        %% Create segment reference lines
        node_corner = [[width height]-(0.5) (-pi/4) -1.61];
        node_top = [0 (height - 1.5 * lane_width) 0 0];
        node_right = [(width - 1.5 * lane_width) 0 (-pi/2) 0];
        node_intersection_top = [0.5*lane_width 0.6 pi/2 0];
        node_intersection_right = [0.6 0.5*lane_width -pi 0];
        node_intersection_centerJ = [0.5*lane_width 0 pi/2 0];
        node_intersection_centerH = [0 0.5*lane_width -pi 0];
        node_intersection_left = [-0.6 -0.5*lane_width 0 0];

        segment_A = solve_segment_path(node_top, node_corner);
        segment_B = solve_segment_path(node_corner, node_right);

        [~,I]=min(abs(segment_A.s - 0.9));
        node_transition_AD = segment_A.nodes(I,:);

        [~,I]=min(abs(segment_B.s - 0.6));
        node_transition_BC = segment_B.nodes(I,:);

        [~,I]=min(abs(segment_A.s - 1.65));
        node_transition_ABEF = segment_A.nodes(I,:);

        node_transition_CF = node_transition_BC + lane_width * [sin(node_transition_BC(3)) -cos(node_transition_BC(3)) 0 0];
        node_transition_DE = node_transition_AD + lane_width * [sin(node_transition_AD(3)) -cos(node_transition_AD(3)) 0 0];

        segment_C = solve_segment_path(node_transition_BC, node_intersection_right);
        segment_D = solve_segment_path(node_intersection_top, node_transition_AD);
        segment_E = solve_segment_path(node_transition_DE, node_transition_ABEF);
        segment_F = solve_segment_path(node_transition_ABEF, node_transition_CF);
        segment_G = solve_segment_path(node_intersection_right + [0 0 2*pi 0], node_intersection_top);
        segment_H = solve_segment_path(node_intersection_right, node_intersection_centerH);
        segment_J = solve_segment_path(node_intersection_centerJ, node_intersection_top);
        segment_K = solve_segment_path(node_intersection_left, node_intersection_top);


        segments = [segment_A segment_B segment_C segment_D segment_E segment_F segment_G segment_H segment_J segment_K];
        save cache
    end   
    

    clf
    hold on
    axis equal
    box on
    grid on
    xlim([-1 1] * full_width*1.1)
    ylim([-1 1] * full_height*1.1)
    plot(full_width*[-1 1 1 -1 -1],full_height*[-1 -1 1 1 -1],'k')

    for i = 1:length(segments)
        plot(segments(i).x(1:10:end), segments(i).y(1:10:end), 'LineWidth', 3)        
    end
    
end

