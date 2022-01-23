

addpath('../../../../../lab_control_center/recording/visualization/');
addpath('../../../../../tools/map_print/map_print2/');


% standard settings
recording_folders = dir('/tmp/cpm_lab_recordings/*');
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

query = 'SELECT rti_json_sample FROM "trajectory@5"';

Trajectories = fetch(conn, query);
Trajectories = Trajectories(~cellfun('isempty',Trajectories));
DataRaw = cellfun(@jsondecode, Trajectories);

lane_graph();
load lane_graph.mat;


    n_path_nodes = 25;
    
for i = 1:length(lane_graph.edges)
    n = size(lane_graph.edges(i).path,1);
    slice = round(linspace(1,n,n_path_nodes));
    lane_graph.edges(i).path = lane_graph.edges(i).path(slice,:);
end

n_lgp = numel(DataRaw(1).lane_graph_positions);
t_start = DataRaw(1).lane_graph_positions(1).estimated_arrival_time.nanoseconds;
x=zeros(n_lgp,1);
y=zeros(n_lgp,1);
t=zeros(n_lgp,1);
for i=1:n_lgp
    edge_index = DataRaw(1).lane_graph_positions(i).edge_index+1;
    edge_path_index = DataRaw(1).lane_graph_positions(i).edge_path_index+1;
    
    x(i) = lane_graph.edges(edge_index).path(edge_path_index,1);
    y(i) = lane_graph.edges(edge_index).path(edge_path_index,2);
    t(i) = DataRaw(1).lane_graph_positions(i).estimated_arrival_time.nanoseconds;
    
end
t = 1e-9*(t-t_start);

figure;
surf([x(:) x(:)], [y(:) y(:)], [t(:) t(:)], ...  % Reshape and replicate data
     'FaceColor', 'none', ...    % Don't bother filling faces with color
     'EdgeColor', 'interp', ...  % Use interpolated color for edges
     'LineWidth', 2);            % Make a thicker line
view(2);   % Default 2-D view
colorbar;  % Add a colorbar
axis equal;xlim([0,4.5]); ylim([0,4]);

figure;
scatter(x,y,[],t,'fill')
axis equal;xlim([0,4.5]); ylim([0,4]);