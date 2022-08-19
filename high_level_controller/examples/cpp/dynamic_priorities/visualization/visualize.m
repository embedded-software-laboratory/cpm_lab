addpath('../../../../../lab_control_center/recording/visualization/');
addpath('../../../../../tools/map_print/map_print2/');

framerate = 30;
resolution = [1920 1080];
frame_per_step = framerate*200;

fig = figure('Visible','Off'...
            ,'Color',[1 1 1]...
            ,'units','pixel'...
            ,'OuterPosition',[100 100 resolution(1)/2 resolution(2)/2]...
);


veh.Length=0.3;
veh.Width=0.12;
active_alpha=0.3;
final_alpha=1;

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
filename = 'testAnimated.gif';


% vehicles to draw
vehicle_ids = [1,2,3,4,5,6];%,7,8,9,10];

% marker settings
mark_vehicles = [];
markers = ['o' '+' '>' 'h' '*' 's'];

active_vehicles = vehicle_ids;
% draw: optimal/actual

sampling_rate = 400 %ms;

plot.fig = figure;

plot.ax = handle(axes); %create an empty axes that fills the figure
set (plot.ax,'NextPlot', 'add');
plot.surfaces = [];     %// create an empty "surface" object
plot.fcas = [];
plot.ids = [];
plot.markers = [];
plot.cars = [];
plot.car_highlight = handle(rectangle())%
axis equal

set (plot.fig, 'position', [10,10,1300,1300]);
view(0,90);
xlim([0,4.5]); ylim([0,4]);
pbaspect([1 1 4])

% Legende:
% Colorbar useful to visualize time without animated vehicles
% colorbar
% cbar = colorbar;
% cbar.Label.String = 'Time [s]'
xlabel('x [m]')
ylabel('y [m]')

% Init empty figure elements
for i=1:length(vehicle_ids)
    plot.surfaces = [ plot.surfaces  handle( surf( NaN(2) ) )];
    plot.markers = [ plot.markers  handle( surf( NaN(2) ) )];
    plot.fcas = [plot.fcas handle( text(0,0,''))];
    plot.ids = [plot.ids handle( text(0,0,''))];
    plot.cars = [plot.cars handle(patch('FaceAlpha',0, 'EdgeAlpha',0))];
end
hold on


%fca table (harrdcoded for 6 vehicles TODO)
colorgen = @(color,text) ['<table border=0 width=400 bgcolor=',color,'><TR><TD>',text,'</TD></TR> </table>'];
fca_table = cell(length(vehicle_ids), length(vehicle_ids));
columns = {'v1', 'v2', 'v3', 'v4', 'v5', 'v6'};
uit = uitable('Position',[330 870 205 130]); %[800 900 300 400]

% step 1: 1644749954800000001; fca working
% step 2: 1644749955200000001; fca not working
videoprefix="s2"

time_step = 1644749955200000001; %DataTraj(1).header.create_stamp.nanoseconds%+ (428*400000000);%+168800000000;% + 90000000000;

s = [];
f_i = 1;
fcas = zeros(length(vehicle_ids), 2);
finals = [];
veh_mi = [];
anim_alpha = repmat(active_alpha,1,max(vehicle_ids));
for iteration=1:length(vehicle_ids)+1   
    free_vehs=vehicle_ids;
    vidname = [char(videoprefix) '_video_iter_' num2str(iteration) '.mp4'];
    v = VideoWriter(fullfile("./video/",vidname),'MPEG-4');
    v.FrameRate = framerate;
    v.Quality = 97;
    open(v);

    fcas = zeros(length(active_vehicles), 2);
    
    i = 1;
    % plot all optimal trajectories and first fca
    for k = active_vehicles
        v_id_index = find(vehicle_ids==k);

        % message index m_i
        m_i = search_trajectory_message_index(DataTraj, time_step, 'Optimal', k);
        
        veh_mi(k) = m_i;% save index for animation
        
        % plot surfaces
        surface = plot_m_i(DataTraj, lane_graph, plot.surfaces(v_id_index), plot.markers(v_id_index), m_i, mark_vehicles, markers);
        
        % annotate fca and ids
        valid_stamp = int64(int64(iteration)*int64(10000000));
        valid_stamp = int64(int64(time_step) + int64(valid_stamp));
        f_i_tmp = search_fca_index(DataFCA, time_step, k, valid_stamp);

        % set(plot.ids(v_id_index), 'Position', [x(1)+0.05 y(1)+0.05 10], 'String', strcat('id=', int2str(DataTraj(m_i).vehicle_id), '; fca=', int2str(DataFCA(f_i_tmp).fca)),'BackgroundColor', 'white');% TODO toggle 3D mode and then supply z coordinate
        % save fca (fca, id)
        fcas(i,:) = [DataFCA(f_i_tmp).fca k]; 
        i = i + 1;
    end
    
   
    % plot finals 
    for k = finals
        delete(s);
        v_id_index = find(vehicle_ids==k);

        % message index m_i
        m_i = search_trajectory_message_index(DataTraj, time_step, 'Final', k);
        veh_mi(k) = m_i; % save index for animation
        plot_m_i(DataTraj, lane_graph, plot.surfaces(v_id_index), plot.markers(v_id_index), m_i, mark_vehicles, markers);
        
        % update annotation (before animation and annotation by alpha channel was introduced)
        % set(plot.ids(v_id_index), 'Position', [x(1)+0.05 y(1)+0.05 10], 'String', strcat('id=', int2str(DataTraj(m_i).vehicle_id), '; Final'));% TODO toggle 3D mode and then supply z coordinate
        % remove fca annotation
        % set(plot.fcas(find(vehicle_ids==k)), 'String', '');
        
        finals = finals(finals~=k)
        anim_alpha(k) = 1;
        plot_car(DataTraj, lane_graph, k,m_i, 1, plot.cars(k), final_alpha,veh.Length, veh.Width);
    end
    
    % update winner of this iteration
    f_i = f_i + length(vehicle_ids) - length(finals) % TODO check
    % sort vehicles by fca
    fcas = sortrows(fcas, 'descend');
    if (size(fcas,1) >= 1)
        % add winner to finals
        finals = [finals fcas(1,2)];
        % remove from active
        active_vehicles = active_vehicles(active_vehicles~=fcas(1,2));
    end

    % table with current state
    for fca_id = 1:size(fcas,1)
        if iteration <= length(vehicle_ids);
            fca_table(iteration, fcas(fca_id,2))= num2cell(fcas(fca_id,1));
        end
    end
    set(uit, 'Data', fca_table);
    uit.ColumnName = columns;
    uit.ColumnEditable = false;
    uit.ColumnWidth = repmat({30},1,length(vehicle_ids));%{30,30,30,30};
    
    % setup of legend 
    if(iteration==1)
        h = zeros(length(vehicle_ids), 1);
        i=1;
        labels = {};
        for id = vehicle_ids
            %h(i) = scatter(NaN,NaN, strcat(markers(find(mark_vehicles==id)), 'k'));
            h(i) = bar(NaN,NaN, 'FaceColor', vehColor(id));
            
            labels = [labels {strcat("Vehicle ", int2str(id))}];
            i=i+1;
        end
        legd = legend(h, labels{:}, 'Position',[0.6 0.67 0.07 0.1]);
    end
    
    %Plot car animations at trajectory t=1
    %save for video
   
    for t_i = 1:length(DataTraj(1).lane_graph_positions)
        for k = vehicle_ids
            [x,y,yaw] = plot_car(DataTraj, lane_graph, k,veh_mi(k), t_i, plot.cars(k), anim_alpha(k), veh.Length, veh.Width);
            % highlight winner when there is one
            if (size(fcas,1) >= 1)
                if k == fcas(1,2)
                    set(plot.car_highlight, ...
                        'Position',[x-((veh.Length/2)+0.1),y-((veh.Length/2)+0.1),veh.Length+0.2,veh.Length+0.2],...
                        'Curvature',[1 1],...
                        'EdgeColor','r',...
                        'LineWidth',3,...
                        'LineStyle', '--');
                end
            else
                set(plot.car_highlight, ...
                        'Position',[x-((veh.Length/2)+0.1),y-((veh.Length/2)+0.1),veh.Length+0.2,veh.Length+0.2],...
                        'Curvature',[1 1],...
                        'EdgeColor',[0,0,0,0],...
                        'LineWidth',3,...
                        'LineStyle', '--');
            end
        end
        frame = getframe(plot.fig);
        writeVideo(v,frame);
        
    end
    close(v)
    save_imgs(plot, iteration, videoprefix);
end


hold off

% save plot at end of iteration 
function save_imgs(plot, iteration, prefix)
    mkdir("video")
    view(0,90);
    frame = getframe(plot.fig);
    [X,Map]=frame2im(frame);
    imwrite(X,strcat('./video/view', prefix,'_i', int2str(iteration), '_0_90.png'));  
end

function surface = plot_m_i(DataRaw, lane_graph, s, marker, m_i, mark_v, markers)
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
         'EdgeColor' vehColor(DataRaw(m_i).vehicle_id), ...  
         'LineWidth' 2};
    if any(mark_v(:) == DataRaw(m_i).vehicle_id)
        %surface = [surface {'Marker' markers(find(mark_v==DataRaw(m_i).vehicle_id)), ...
        %                    'DisplayName', strcat('vehicle id ', int2str(DataRaw(m_i).vehicle_id))}]
        mark = { 'XData'  [x(1:8:end) x(1:8:end)]  'YData' [y(1:8:end) y(1:8:end)]  'ZData',[t(1:8:end) t(1:8:end)] ... 
         'FaceColor', 'interp',...%vehColor(DataRaw(m_i).vehicle_id), ...    
         'EdgeColor' 'interp', ...  % Use interpolated color for edges
         'LineStyle' 'none', ...
         'LineWidth' 2, ...
         'Marker' markers(find(mark_v==DataRaw(m_i).vehicle_id)), ...
         'MarkerSize' 11, ...
         'DisplayName', strcat('vehicle id ', int2str(DataRaw(m_i).vehicle_id))...    
         ,'MarkerFaceColor', 'none'...
         %,'MarkerEdgeColor', vehColor(DataRaw(m_i).vehicle_id)...
         };
        set (marker, mark{:});
    end
    set ( s, surface{:});  
end

function [x, y, t] = get_start_pos(DataTraj, lane_graph, m_i)
        edge_index = DataTraj(m_i).lane_graph_positions(1).edge_index+1;
        edge_path_index = DataTraj(m_i).lane_graph_positions(1).edge_path_index+1;
        
        x = lane_graph.edges(edge_index).path(edge_path_index,1);
        y = lane_graph.edges(edge_index).path(edge_path_index,2);
        t = DataTraj(m_i).lane_graph_positions(1).estimated_arrival_time.nanoseconds;
end

% Plots car with id k and trajectory message m_i at time t_i of the
% trajectory
function [x,y,yaw]=plot_car(DataTraj, lane_graph, k,m_i, t_i, car, alpha, length, width)

    % Plot car as rectangle
    [x,y,yaw] = car_position(DataTraj, lane_graph, m_i, t_i);
    vehiclePolygon = transformedRectangle(x,y,yaw, length,width);
    set(car, 'XData',vehiclePolygon(1,:)...
            ,'YData', vehiclePolygon(2,:)...
            , 'FaceColor', vehColor(k)...
            ,'LineWidth', 0.001 ...
            ,'EdgeAlpha', alpha...
            ,'FaceAlpha', alpha...
    );
end