

addpath('../../../../../lab_control_center/recording/visualization/');
addpath('../../../../../tools/map_print/map_print2/');


% standard settings
recording_folders = dir('./cpm_lab_recordings/*');
current_folder = recording_folders(end);
assert(current_folder.isdir);
recording_file = fullfile(...
    current_folder.folder, ...
    current_folder.name, ...
    'recording.dat' ...
);
dds_domain = getenv("DDS_DOMAIN");
        
        
assert(isfile(recording_file));
conn = sqlite(recording_file);

query = 'SELECT rti_json_sample FROM "trajectory@21"'; % TODO change 21 to dds_domain

Trajectories = fetch(conn, query);
Trajectories = Trajectories(~cellfun('isempty',Trajectories));
DataTraj = cellfun(@jsondecode, Trajectories);
query = 'SELECT rti_json_sample FROM "futureCollisionAssessment@21"'; % TODO change 21 to dds_domain

FutureCollisionAssessments = fetch(conn, query);
FutureCollisionAssessments = FutureCollisionAssessments(~cellfun('isempty',FutureCollisionAssessments));
DataFCA = cellfun(@jsondecode, FutureCollisionAssessments);

lane_graph();
load lane_graph.mat;

n_path_nodes = 25;
    
for i = 1:length(lane_graph.edges)
    n = size(lane_graph.edges(i).path,1);
    slice = round(linspace(1,n,n_path_nodes));
    lane_graph.edges(i).path = lane_graph.edges(i).path(slice,:);
end

% GOAL:
% 1. draw all optimal trajs
% 2. annotate trajs with id
% 3. for veh 1..n (1) highlight collisions; (2) annotate fca; (3) replace
% optimal with actual trajectory
% 4. show deadlock?
filename = 'testAnimated.gif';

% vehicles to draw
vehicle_ids = [1,2,3,4,5,6,7,8,9,10];

% marker settings
mark_vehicles = [1 2 3 4];
markers = ['o' '+' '>' 'h' '*'];

active_vehicles = vehicle_ids;
% draw: optimal/actual

sampling_rate = 400 %ms;

plot.fig = figure;

plot.ax   = handle(axes) ;                 %// create an empty axes that fills the figure
set (plot.ax,'NextPlot', 'add');
plot.surfaces = [];     %// create an empty "surface" object
plot.fcas = [];
plot.ids = [];

for i=1:length(vehicle_ids)
    plot.surfaces = [ plot.surfaces  handle( surf( NaN(2) ) )];
    plot.fcas = [plot.fcas handle( text(0,0,''))];
    plot.ids = [plot.ids handle( text(0,0,''))];
end

hold on
time_step = DataTraj(1).header.create_stamp.nanoseconds%+ (428*400000000);%+168800000000;% + 90000000000;

f_i = 1;
fcas = zeros(length(vehicle_ids), 2);
finals = [];
for iteration=1:length(vehicle_ids)+1   
    i = 1;
    % plot all optimal trajectories and first fca
    for k = active_vehicles
        v_id_index = find(vehicle_ids==k);

        % message index m_i
        m_i = search_m_i(DataTraj, time_step, 'Optimal', k);
        
        % plot surfaces
        surface = plot_m_i(DataTraj, lane_graph, plot.surfaces(v_id_index), m_i, mark_vehicles, markers);
    
        [x, y, z] = get_start_pos(DataTraj, lane_graph, m_i);
        % annotate fca
        valid_stamp = int64(int64(iteration)*int64(10000000));
        valid_stamp = int64(int64(time_step) + int64(valid_stamp));
        f_i_tmp = search_f_i(DataFCA, time_step, k, valid_stamp);

        % annotate ids
        set(plot.ids(v_id_index), 'Position', [x(1) y(1) 0], 'String', strcat('id=', int2str(DataTraj(m_i).vehicle_id), '; fca=', int2str(DataFCA(f_i_tmp).fca)));% TODO toggle 3D mode and then supply z coordinate
        
        %text(x(1)+0.2, y(1)+0.2, strcat('fca=', int2str(DataFCA(f_i_tmp).fca))) % TODO toggle 3D mode and then supply z coordinate
        
        % save fca (fca, id)
        fcas(i,:) = [DataFCA(f_i_tmp).fca k]; 
        i = i + 1;
    end
    
    % plot finals 
    for k = finals
        v_id_index = find(vehicle_ids==k);
        
        % message index m_i
        m_i = search_m_i(DataTraj, time_step, 'Final', k);
        plot_m_i(DataTraj, lane_graph, plot.surfaces(v_id_index), m_i, mark_vehicles, markers);
        % update annotation
        [x, y, z] = get_start_pos(DataTraj, lane_graph, m_i);
        set(plot.ids(v_id_index), 'Position', [x(1) y(1) 0], 'String', strcat('id=', int2str(DataTraj(m_i).vehicle_id), '; Final'));% TODO toggle 3D mode and then supply z coordinate
        % remove fca annotation
         set(plot.fcas(find(vehicle_ids==k)), 'String', '');
    end
    % and remaining new fcas TODO
    
    % update winner of this iteration
    f_i = f_i + length(vehicle_ids) - length(finals) % TODO check
    % sort vehicles by fca
    fcas = sortrows(fcas, 'descend');
    % add winner to finals
    finals = [finals fcas(1,2)];
    % remove from active
    active_vehicles = active_vehicles(active_vehicles~=fcas(1,2));
    
    pause()
    % Capture the plot as an image 
    frame = getframe(plot.fig); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if iteration == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 
end

%view(2);   % Default 2-D view
colorbar;  % Add a colorbar
axis equal;xlim([0,4.5]); ylim([0,4]);
%plot_collisions(plot.surfaces);
hold off

% searches trajectory message
function m_i = search_m_i(DataTraj, time, type, vehicle_id) % type='Optimal' or type='Final'
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

% searches fca message (starting at a given index)
function f_i = search_f_i(DataFCA, time, vehicle_id, valid_stamp)
    f_i = 1;
    fca = DataFCA(f_i);
    while(~((int64(fca.header.valid_after_stamp.nanoseconds) < valid_stamp+10000 && ...
            int64(fca.header.valid_after_stamp.nanoseconds) > valid_stamp-10000 ) ...
            & (fca.vehicle_id == vehicle_id)) & (f_i <= numel(DataFCA)))
    f_i = f_i + 1;
        if(f_i <= numel(DataFCA))
            fca = DataFCA(f_i);
        else 
            disp(['Error: Message not found! ' ...
                'v_id: ' int2str(vehicle_id) ...
                ' valid_stamp: ' int2str(valid_stamp) ...
                ' time: ' int2str(time)]);
        end
    end
    disp(['v_id:' int2str(vehicle_id) ...
        ' FCA: ' int2str(fca.fca) ...
        ' f_i:' int2str(f_i) ...
        ' fca stamp: ' int2str(fca.header.valid_after_stamp.nanoseconds) ...
        ' time: ' int2str(time) ...
        ' valid_stamp: ' int2str(valid_stamp)])
end

function update_traj(DataRaw)
end

function surface = plot_m_i(DataRaw, lane_graph, s, m_i, mark_v, markers)

    n_lgp = numel(DataRaw(m_i).lane_graph_positions);
    t_start = DataRaw(m_i).lane_graph_positions(1).estimated_arrival_time.nanoseconds;

    % Output used id
    
    disp(['plotting id:', num2str(DataRaw(m_i).vehicle_id), ' type:', DataRaw(m_i).type])
    
    % x,y positions of vehicle DataRaw.vehicle_id; t estimated arrival of the vehicle at (x,y)
    x=zeros(n_lgp,1);
    y=zeros(n_lgp,1);
    t=zeros(n_lgp,1);
    
    
    for i=1:n_lgp
        edge_index = DataRaw(m_i).lane_graph_positions(i).edge_index+1;
        edge_path_index = DataRaw(m_i).lane_graph_positions(i).edge_path_index+1;
        
        x(i) = lane_graph.edges(edge_index).path(edge_path_index,1);
        y(i) = lane_graph.edges(edge_index).path(edge_path_index,2);
        t(i) = DataRaw(m_i).lane_graph_positions(i).estimated_arrival_time.nanoseconds;
        
    end
    t = 1e-9*(t-t_start);
    
    surface = { 'XData'  [x(:) x(:)]  'YData' [y(:) y(:)]  'ZData',[t(:) t(:)] ... 
         'FaceColor' 'none', ...    % Don't bother filling faces with color
         'EdgeColor' 'interp', ...  % Use interpolated color for edges
         'LineWidth' 2};
    if any(mark_v(:) == DataRaw(m_i).vehicle_id)
        surface = [surface {'Marker' markers(find(mark_v==DataRaw(m_i).vehicle_id))}]
    end
    set ( s, surface{:});
    %     surf([x(:) x(:)], [y(:) y(:)], [t(:) t(:)], ...  % Reshape and replicate data
%          'FaceColor', 'none', ...    % Don't bother filling faces with color
%          'EdgeColor', 'interp', ...  % Use interpolated color for edges
%          'LineWidth', 2);            % Make a thicker line
    
end

function [x, y, t] = get_start_pos(DataTraj, lane_graph, m_i)
        edge_index = DataTraj(m_i).lane_graph_positions(1).edge_index+1;
        edge_path_index = DataTraj(m_i).lane_graph_positions(1).edge_path_index+1;
        
        x = lane_graph.edges(edge_index).path(edge_path_index,1);
        y = lane_graph.edges(edge_index).path(edge_path_index,2);
        t = DataTraj(m_i).lane_graph_positions(1).estimated_arrival_time.nanoseconds;
end


function plot_collisions(surfaces)
    for i = 1:length(surfaces)
        for j = i:length(surfaces)
            intersections(surfaces(i).XData,surfaces(i).YData, surfaces(j).XData, surfaces(j).YData )
        end
    end
end
% 
% figure;
% scatter(x,y,[],t,'fill')
% axis equal;xlim([0,4.5]); ylim([0,4]);