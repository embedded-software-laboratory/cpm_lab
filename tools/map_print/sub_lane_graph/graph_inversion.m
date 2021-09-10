function lane_graph = graph_inversion(original_lane_graph)
lane_graph = original_lane_graph;

end_node_index = num2cell([original_lane_graph.edges.end_node_index]);
[lane_graph.edges.start_node_index] = end_node_index{:};

start_node_index = num2cell([original_lane_graph.edges.start_node_index]);
[lane_graph.edges.end_node_index] = start_node_index{:};

for i_edge = 1:numel(lane_graph.edges)
    lane_graph.edges(i_edge).path = flip(original_lane_graph.edges(i_edge).path);
end
end

