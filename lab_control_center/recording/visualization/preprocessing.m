% denesting resolves the nested input structs created by decoding the
% JSON-formatted samples of rti DDS recordings by function dbConnection()  
% to allow ready accessibilty when plotting.
% It creates the struct DataByVehicle with first level fields
% distinguishing between the vehicles, seconds level fields 'observation' 
% and 'state' corresponding to the topic of rti DDS the data were recorded 
% from and third level fields referring to the data measured. Labels from
% original recording are kept as fieldnames.

function [DataByVehicle] = preprocessing(dds_domain, recording_file)

% fetch samples
ddsJsonSample = dbConnection(dds_domain, recording_file);

t_start_list = [];

VehicleStateTable = [];
VehicleObservationTable = [];
VehicleCommandPathTrackingTable = [];

HeaderState = [];
HeaderObservation = [];
HeaderCommandPathTracking = [];


try
[VehicleStateTable, HeaderState, t_start_state_nanos] = jsonDeconstruct(ddsJsonSample.VehicleState);
t_start_list = [t_start_list, t_start_state_nanos];
catch

end

try
[VehicleObservationTable, HeaderObservation, t_start_observation_nanos] = jsonDeconstruct(ddsJsonSample.VehiclePose);
t_start_list = [t_start_list, t_start_observation_nanos];
catch

end

try
[VehicleCommandPathTrackingTable, HeaderCommandPathTracking, t_start_pathtracking_nanos] = jsonDeconstruct(ddsJsonSample.VehicleCommandPathTracking);
t_start_list = [t_start_list, t_start_pathtracking_nanos];
catch

end

t_start_nanos = min(t_start_list);

%% Sort data by vehicle ids, resolve nested structures and store into struct DataByVehicle.

DataByVehicle = struct;

for iVeh = 1:max([VehicleStateTable.vehicle_id]) % Loop over vehicle ids.

    % Read out and store nested data of vehicle observation filtered by
    % current vehicle id via logical table operation.
    try
    currentRowsObservation = VehicleObservationTable.vehicle_id == iVeh; 

    DataByVehicle(iVeh).observation.create_stamp_nanos = [HeaderObservation.create_stamp(currentRowsObservation).nanoseconds]';
    DataByVehicle(iVeh).observation.valid_after_stamp_nanos = [HeaderObservation.valid_after_stamp(currentRowsObservation).nanoseconds]';
    DataByVehicle(iVeh).observation.create_stamp = 1e-9 * (DataByVehicle(iVeh).observation.create_stamp_nanos-t_start_nanos);
    DataByVehicle(iVeh).observation.valid_after_stamp = 1e-9 * (DataByVehicle(iVeh).observation.valid_after_stamp_nanos-t_start_nanos);
    DataByVehicle(iVeh).observation.x = [VehicleObservationTable.pose(currentRowsObservation).x]';
    DataByVehicle(iVeh).observation.y = [VehicleObservationTable.pose(currentRowsObservation).y]';
    DataByVehicle(iVeh).observation.yaw = [VehicleObservationTable.pose(currentRowsObservation).yaw]';
    catch
    end

    % Read out and store nested data of vehicle state filtered by
    % current vehicle id via logical table operation.
    try
    currentRowsState = VehicleStateTable.vehicle_id == iVeh;
    DataByVehicle(iVeh).state.create_stamp_nanos = [HeaderState.create_stamp(currentRowsState).nanoseconds]';
    DataByVehicle(iVeh).state.valid_after_stamp_nanos = [HeaderState.valid_after_stamp(currentRowsState).nanoseconds]';
    DataByVehicle(iVeh).state.create_stamp = 1e-9 * (DataByVehicle(iVeh).state.create_stamp_nanos-t_start_nanos);
    DataByVehicle(iVeh).state.valid_after_stamp = 1e-9 * (DataByVehicle(iVeh).state.valid_after_stamp_nanos-t_start_nanos);
    DataByVehicle(iVeh).state.x = [VehicleStateTable.pose(currentRowsState).x]';
    DataByVehicle(iVeh).state.y = [VehicleStateTable.pose(currentRowsState).y]';
    DataByVehicle(iVeh).state.yaw = [VehicleStateTable.pose(currentRowsState).yaw]';
    
    % Read out and store directly accessible data of vehicle state filtered by
    % current vehicle id via logical table operation.
    for jFields = 4:width(VehicleStateTable)
        currentField = VehicleStateTable.Properties.VariableNames{jFields};
        DataByVehicle(iVeh).state.(currentField) = VehicleStateTable.(currentField)(currentRowsState);
    end
    catch
    end
    
    % Read out and store nested data of vehicle path tracking commands filtered by
    % current vehicle id via logical table operation.
    try
    currentRowsPathTracking = VehicleCommandPathTrackingTable.vehicle_id == iVeh;
    DataByVehicle(iVeh).pathtracking.create_stamp_nanos = [HeaderCommandPathTracking.create_stamp(currentRowsPathTracking).nanoseconds]';
    DataByVehicle(iVeh).pathtracking.valid_after_stamp_nanos = [HeaderCommandPathTracking.valid_after_stamp(currentRowsPathTracking).nanoseconds]';
    DataByVehicle(iVeh).pathtracking.create_stamp = 1e-9 * (DataByVehicle(iVeh).pathtracking.create_stamp_nanos-t_start_nanos);
    DataByVehicle(iVeh).pathtracking.valid_after_stamp = 1e-9 * (DataByVehicle(iVeh).pathtracking.valid_after_stamp_nanos-t_start_nanos);
    
    % TODO: Array with path points
    allPTCommands = VehicleCommandPathTrackingTable.path(currentRowsPathTracking);
    path_table = [];
    for i = 1:numel(allPTCommands)
        path_struct = VehicleCommandPathTrackingTable.path(i);
        path_list = [];
        s = [path_struct{1}(:).s];
        pose = [path_struct{1}(:).pose];
        x = [pose(:).x];
        y = [pose(:).y];
        yaw = [pose(:).yaw];

        for i=1:numel(s)
            pp = PathPoint;
            pp.s = s(i);
            pp.pose.x = x(i);
            pp.pose.y = y(i);
            pp.pose.yaw = yaw(i);
            path_list = [path_list pp];
        end
        path_table = [path_table; path_list];
    end
    DataByVehicle(iVeh).pathtracking.path = path_table;
    DataByVehicle(iVeh).pathtracking.speed = [VehicleCommandPathTrackingTable.speed(currentRowsPathTracking)]';
    catch
        % continue
    end
end
end


function [Table, Header, t_start_nanos] = jsonDeconstruct(JSONData)
%% Parse {JSON} formatted sample into structs for further processing.
DataRaw = cellfun(@jsondecode, JSONData);

%% Create tables from decoded structs to enable filtering by logical operation and vectorized access of nested fields.
Table = struct2table(DataRaw);

% Create tables from nested struct field 'header'.
Header = struct2table([DataRaw.header]);

%% Find timepoint zero, i.e. smallest timestamp in recording
t_start_nanos = min([Header.create_stamp.nanoseconds]);
end
