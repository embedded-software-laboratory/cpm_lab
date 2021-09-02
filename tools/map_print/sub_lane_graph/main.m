function main
    %% Create lane_graph.mat
    close all
    addpath('../map_print2');
    assert(isfile('../map_print2/cache.mat'));
    if (~isfile('../map_print2/lane_graph.mat'))
        cur_dir = pwd;
        cd('../map_print2');
        lane_graph();
        cd(cur_dir);
    end
    org = load('lane_graph.mat');
    original_lane_graph = org.lane_graph;
    visualize_lane_graph(original_lane_graph);
    
    %% delete nodes
%     outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
%     inner_cross_nodes = [37,  40, 28, 14, 11,57,  24, 68, 46, 75, 78,  66, 48, 23, 42,  60,  61, 27, 16, 17];
%     comb = union(outer_circle_nodes, inner_cross_nodes);
%     steep_curve_edges = [7 13; 58 55; 49 62; 79 69; 73 77; 38 33; 29 43; 19 2];

comb = [47, 65];
%     inner_cross_nodes = [37,  40, 28, 14, 11,57,  24, 68, 46, 75, 78,  66, 48, 23, 42,  60,  61, 27, 16, 17];
%     comb = union(outer_circle_nodes, inner_cross_nodes);
     steep_curve_edges = [47 67];
        
    laneGraph = sub_graph_deletion(original_lane_graph, comb, steep_curve_edges);
    visualize_lane_graph(laneGraph);
    lane_graph_to_cpp(laneGraph)
       
    %% select nodes
%     outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
%     
%     laneGraph = sub_graph_selection(original_lane_graph, outer_circle_nodes);
%     visualize_lane_graph(laneGraph);
%     lane_graph_to_cpp(laneGraph);
    
    %% invert graph
%     inverted_laneGraph = graph_inversion(laneGraph);
%     lane_graph_to_cpp(inverted_laneGraph);
end