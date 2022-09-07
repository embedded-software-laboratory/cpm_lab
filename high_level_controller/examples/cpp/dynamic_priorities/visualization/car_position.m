function [x,y,yaw] = car_position(DataTraj, lane_graph, m_i, t_i)
        edge_index = DataTraj(m_i).lane_graph_positions(t_i).edge_index+1;
        edge_path_index = DataTraj(m_i).lane_graph_positions(t_i).edge_path_index+1;
        
        
        x = lane_graph.edges(edge_index).path(edge_path_index,1);
        y = lane_graph.edges(edge_index).path(edge_path_index,2);
        
        yaw = atan(lane_graph.edges(edge_index).path(edge_path_index,4)...
                   / lane_graph.edges(edge_index).path(edge_path_index,3));
end

