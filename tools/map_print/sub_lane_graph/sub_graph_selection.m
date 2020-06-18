% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

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

