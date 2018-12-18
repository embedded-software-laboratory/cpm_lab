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
        intersection_radius = 0.8;

        %% Create segment reference lines
        node_corner = [[width height]-(0.5) (-pi/4) -1.61];
        node_top = [0 (height - 1.5 * lane_width) 0 0];
        node_right = [(width - 1.5 * lane_width) 0 (-pi/2) 0];
        node_intersection_top = [0.5*lane_width intersection_radius pi/2 0];
        node_intersection_right = [intersection_radius 0.5*lane_width -pi 0];
        node_intersection_centerJ = [0.5*lane_width 0 pi/2 0];
        node_intersection_centerH = [0 0.5*lane_width -pi 0];
        node_intersection_left = [-intersection_radius -0.5*lane_width 0 0];

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

        save cache
    end
    
    segment_A.lane_boundaries(1) = struct('offset', -0.5, 'style', '-'  , 'width', 2);
    segment_A.lane_boundaries(2) = struct('offset',  0.5, 'style', '--', 'width', 1);
    segment_A.lane_boundaries(3) = struct('offset',  1.5, 'style', '-'  , 'width', 2);
    
    segment_B.lane_boundaries(1) = struct('offset', -0.5, 'style', '-'  , 'width', 2);
    segment_B.lane_boundaries(2) = struct('offset',  0.5, 'style', '--', 'width', 1);
    segment_B.lane_boundaries(3) = struct('offset',  1.5, 'style', '-'  , 'width', 2);
    
    segment_C.lane_boundaries(1) = struct('offset', -1.5, 'style', '-'  , 'width', 2);
    segment_C.lane_boundaries(2) = struct('offset', -0.5, 'style', '--', 'width', 1);
    segment_C.lane_boundaries(3) = struct('offset',  0.5, 'style', '-'  , 'width', 2);
    
    segment_D.lane_boundaries(1) = struct('offset', -1.5, 'style', '-'  , 'width', 2);
    segment_D.lane_boundaries(2) = struct('offset', -0.5, 'style', '--', 'width', 1);
    segment_D.lane_boundaries(3) = struct('offset',  0.5, 'style', '-'  , 'width', 2);
    
    segment_E.lane_boundaries(1) = struct('offset', -0.5, 'style', '--', 'width', 1);
    segment_E.lane_boundaries(2) = struct('offset', 0.5, 'style', '--', 'width', 1);
    
    segment_F.lane_boundaries(1) = struct('offset', -0.5, 'style', '--', 'width', 1);
    segment_F.lane_boundaries(2) = struct('offset', 0.5, 'style', '--', 'width', 1);
    
    segment_G.lane_boundaries(1) = struct('offset', -1.5, 'style', '-', 'width', 2);
    segment_G.lane_boundaries(2) = struct('offset', -0.5, 'style', ':', 'width', 1);
    segment_G.lane_boundaries(3) = struct('offset',  0.5, 'style', ':', 'width', 1);
    segment_G.lane_boundaries(4) = struct('offset',  1.5, 'style', ':', 'width', 1);
    segment_G.lane_boundaries(5) = struct('offset',  2.5, 'style', ':', 'width', 1);
    
    segment_H.lane_boundaries(1) = struct('offset', -1.5, 'style', '', 'width', 2);
    segment_H.lane_boundaries(2) = struct('offset', -0.5, 'style', ':', 'width', 1);
    segment_H.lane_boundaries(3) = struct('offset',  0.5, 'style', '', 'width', 2);
    
    segment_J.lane_boundaries(1) = struct('offset', -1.5, 'style', '', 'width', 2);
    segment_J.lane_boundaries(2) = struct('offset', -0.5, 'style', ':', 'width', 1);
    segment_J.lane_boundaries(3) = struct('offset',  0.5, 'style', '', 'width', 2);
    
    
    
    close all
    figure('Visible','off')
    hold on

    plot_scale = 1000 / 25.4 * 72; % 1:1 scale
    plot_scale = 100 / 25.4 * 72; % 1:10 scale
    plot_scale = 50 / 25.4 * 72; % 1:20 scale
    
    segments = [segment_A segment_B segment_C segment_D segment_E segment_F segment_G segment_H segment_J];
    
    slice = round(linspace(1,length(segment_A.s),300));
    clf
    hold on
    axis equal
    box on
    xlim(plot_scale*[-1 1] * full_width)
    ylim(plot_scale*[-1 1] * full_height)
    
    for sx = [-1 1]
        for sy = [-1 1]
            if 1%~(sx == 1 && sy == 1)
                for i = 1:length(segments)
                    for j = 1:length(segments(i).lane_boundaries)
                        
                        x = segments(i).x(slice);
                        y = segments(i).y(slice);
                        yaw = segments(i).yaw(slice);
                        c = segments(i).lane_boundaries(j).offset * lane_width * cos(yaw);
                        s = segments(i).lane_boundaries(j).offset * lane_width * sin(yaw);
                        plot(plot_scale*sx*(x-s), plot_scale*sy*(y+c), segments(i).lane_boundaries(j).style, 'LineWidth', segments(i).lane_boundaries(j).width * plot_scale * 0.005, 'Color', [1 1 1])
                    end
                end
            end
        end
    end
    
%     for i = 1:length(segments)
%         plot(segments(i).x(slice), segments(i).y(slice), 'LineWidth', 5)        
%     end
    

    axis off
    ax = gca;
    ax.Position = [0 0 1 1];
    fig = gcf;
    fig.PaperPositionMode = 'manual';
    fig.PaperType = '<custom>';
    fig.PaperUnits = 'points';
    fig.PaperSize = 2*[full_width full_height] * plot_scale;
    fig.InvertHardcopy = 'off';
    fig.Color = 'black';
    fig.PaperPosition = [0 0 2*full_width 2*full_height] * plot_scale;

    print(fig, 'x.pdf', '-dpdf', '-painters')
end

