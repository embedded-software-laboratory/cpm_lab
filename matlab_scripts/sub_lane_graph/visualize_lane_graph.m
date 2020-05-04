function visualize_lane_graph(lane_graph)
n_nodes = size(lane_graph.nodes,1);
figure
hold on
for i_node = 1:n_nodes
    plot(lane_graph.nodes(i_node,1),lane_graph.nodes(i_node,2),'bo');
    textstr = num2str(i_node);
    text(lane_graph.nodes(i_node,1),lane_graph.nodes(i_node,2),textstr);
end
n_edges = size(lane_graph.edges,2);
for i_edge = 1:n_edges
    plot(lane_graph.edges(i_edge).path(:,1), ...
        lane_graph.edges(i_edge).path(:,2), ...
        'r');
end