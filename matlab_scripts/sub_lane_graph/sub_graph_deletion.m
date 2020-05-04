% Input del_nodes: Array of nodes to be deleted
%       del_egdges: Array of node tuples [start,end] to be deleted
% Output changed_graph.mat with lane_graph according to deletions
function lane_graph = sub_graph_deletion(del_nodes, del_edges)

    org = load('lane_graph.mat');
    lane_graph = org.lane_graph;
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

    save changed_graph lane_graph
    
end