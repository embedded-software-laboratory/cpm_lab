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

function lane_graph = sub_graph_deletion(original_lane_graph, del_nodes, del_edges)
% Input del_nodes: Array of nodes to be deleted
%       del_egdges: Array of node tuples [start,end] to be deleted
% Output changed_graph.mat with lane_graph according to deletions
    lane_graph = original_lane_graph;
    n_nodes = size(lane_graph.nodes, 1);
    n_edges = size(lane_graph.edges, 2);
        
    
    set_nodes = [1:size(lane_graph.nodes, 1)];
    set_diff = setdiff(set_nodes, del_nodes);
    
    % Delete edges related to del_nodes and edges from del_edges first
    for i = n_edges:-1:1
        if(ismember(lane_graph.edges(i).start_node_index, del_nodes) || ismember(lane_graph.edges(i).end_node_index, del_nodes))
            lane_graph.edges(i) = [];
        else
            for j = 1:size(del_edges, 1)
                if(ismember(lane_graph.edges(i).start_node_index, del_edges(j, 1)) && ismember(lane_graph.edges(i).end_node_index, del_edges(j, 2)))
                    lane_graph.edges(i) = [];
                end
            end
        end
    end
        
    n_edges_after = size(lane_graph.edges, 2);
    
    % Delete nodes in del_nodes
    for i = n_nodes:-1:1
        if(ismember(i, del_nodes))
            lane_graph.nodes(i,:) = []; 
            
            % Shift edge indices if deleted node was lower
            for j = 1:n_edges_after
                % Check for start index
                if(lane_graph.edges(j).start_node_index > i)
                    lane_graph.edges(j).start_node_index = lane_graph.edges(j).start_node_index - 1;
                end
                % Check for end index
                if(lane_graph.edges(j).end_node_index > i)
                    lane_graph.edges(j).end_node_index = lane_graph.edges(j).end_node_index - 1;
                end
            end
        end
    end
        
    n_nodes_after = size(lane_graph.nodes, 1);  
    
    figure
    hold on
    
    scatter(lane_graph.nodes(:,1), lane_graph.nodes(:,2))
    for i = 1:n_nodes_after
        text(lane_graph.nodes(i,1), lane_graph.nodes(i,2),strcat(' ', num2str(set_diff(i))));
    end
    
    for i = 1:n_edges_after
        plot(lane_graph.edges(i).path(:,1), lane_graph.edges(i).path(:,2))
    end
end