% searches trajectory message
function m_i = search_trajectory_message_index(DataTraj, time, type, vehicle_id) % type='Optimal' or type='Final'
    m_i = 1; % message index
    trajectory = DataTraj(m_i);
    while(~(trajectory.header.create_stamp.nanoseconds == time & strcmp(trajectory.type, type) & trajectory.vehicle_id == vehicle_id) & m_i <= numel(DataTraj))
        m_i = m_i + 1;
        if(m_i <= numel(DataTraj))
            trajectory = DataTraj(m_i);
        else 
            disp('Error: Message not found!')
        end
    end
end