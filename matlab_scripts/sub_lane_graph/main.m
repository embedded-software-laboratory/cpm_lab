function main
    %% delete nodes
    outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
    inner_cross_nodes = [37,  40, 28, 14, 11,57,  24, 68, 46, 75, 78,  66, 48, 23, 42,  60,  61, 27, 16, 17];
    comb = union(outer_circle_nodes, inner_cross_nodes);
    steep_curve_edges = [7 13; 58 55; 49 62; 79 69; 73 77; 38 33; 29 43; 19 2];
    
    addpath('../map_print/map_print2');
    
    lane_graph = sub_graph_deletion(comb, steep_curve_edges);
    visualize_lane_graph(lane_graph);
    lane_graph_to_cpp(lane_graph)
       
    %% select nodes
    addpath('../map_print/map_print2');
    outer_circle_nodes = [3, 4, 6, 8, 10, 56, 54, 51, 52, 70, 72, 74, 36, 34, 32, 30];
    lane_graph = sub_graph_selection(outer_circle_nodes);
    visualize_lane_graph(lane_graph);
    lane_graph_to_cpp(lane_graph);
end