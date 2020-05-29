function lane_graph = sub_graph_selection(original_lane_graph, selected_nodes)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
lane_graph.nodes = original_lane_graph.nodes(selected_nodes,:);
lane_graph.edges = [];
n_edges = size(original_lane_graph.edges,2);
for i_edge = 1:n_edges
    % Check if edge connects two selected nodes
    current_edge = original_lane_graph.edges(i_edge);
    start_node = current_edge.start_node_index;
    end_node = current_edge.end_node_index;
    is_start_node_sel = any(selected_nodes == start_node);
    is_end_node_sel = any(selected_nodes == end_node);
    if (is_start_node_sel && is_end_node_sel)
        edge = current_edge;
        edge.start_node_index = find(selected_nodes == start_node);
        edge.end_node_index = find(selected_nodes == end_node);
        lane_graph.edges = [lane_graph.edges edge];
    end
end

