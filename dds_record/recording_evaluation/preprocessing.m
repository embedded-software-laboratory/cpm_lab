% denesting resolves the nested input structs created by decoding the
% JSON-formatted samples of rti DDS recordings by function dbConnection()  
% to allow ready accessibilty when plotting.
% It creates the struct DataByVehicle with first level fields
% distinguishing between the vehicles, seconds level fields 'observation' 
% and 'state' corresponding to the topic of rti DDS the data were recorded 
% from and third level fields referring to the data measured. Labels from
% original recording are kept as fieldnames.

function [DataByVehicle] = preprocessing(dds_domain, recording_file)
    % todo Timestmaps normieren ueber vehicles
    function [secondStamps] = timestampConversion(unixStamps)
        % Read delta t from unix timestamp and convert into seconds
        secondStamps = unixStamps;
        secondStamps(2:end) = cumsum(unixStamps(2:end)-unixStamps(1:end-1))/1000000000;
        secondStamps(1) = 0;
    end
%% Load dds sample if it exists, else call function dbConnection to fetch it.
% if exist('dds_json_sample.mat', 'file') 
%    load('dds_json_sample', 'ddsVehicleStateJsonSample', 'ddsVehiclePoseJsonSample')
% else 
%    [ddsVehicleStateJsonSample, ddsVehiclePoseJsonSample] = dbConnection(dds_domain, recording_file);
% end
[ddsVehicleStateJsonSample, ddsVehiclePoseJsonSample] = dbConnection(dds_domain, recording_file);
%% Parse {JSON} formatted sample into structs for further processing.
VehicleStateRaw = cellfun(@jsondecode, ddsVehicleStateJsonSample);
VehicleObservationRaw = cellfun(@jsondecode, ddsVehiclePoseJsonSample);

%% Create tables from decoded structs to enable filtering by logical operation and vectorized access of nested fields.
VehicleStateTable = struct2table(VehicleStateRaw);
VehicleObservationTable = struct2table(VehicleObservationRaw);

% Create tables from nested struct field 'header'.
HeaderObservation = struct2table([VehicleObservationRaw.header]);
HeaderState = struct2table([VehicleStateRaw.header]);

%% Sort data by vehicle ids, resolve nested structures and store into struct DataByVehicle.

DataByVehicle = struct;

for iVehicles = 1:max([VehicleStateRaw.vehicle_id]) % Loop over vehicle ids.

    % Build logical array of all rows in data tables corresponding to current vehicle.
    currentRowsState = VehicleStateTable.vehicle_id == iVehicles;
    currentRowsObservation = VehicleObservationTable.vehicle_id == iVehicles; 

    currentVehicle = ['vehicle_' int2str(iVehicles)]; % Create fieldname from current vehicle id.

    % Read out and store nested data of vehicle observation filtered by
    % current vehicle id via logical table operation.

    DataByVehicle.(currentVehicle).observation.create_stamp = timestampConversion([HeaderObservation.create_stamp(currentRowsObservation).nanoseconds]');
    DataByVehicle.(currentVehicle).observation.valid_after_stamp = timestampConversion([HeaderObservation.valid_after_stamp(currentRowsObservation).nanoseconds]');
    DataByVehicle.(currentVehicle).observation.x = [VehicleObservationTable.pose(currentRowsObservation).x]';
    DataByVehicle.(currentVehicle).observation.y = [VehicleObservationTable.pose(currentRowsObservation).y]';
    DataByVehicle.(currentVehicle).observation.yaw = [VehicleObservationTable.pose(currentRowsObservation).yaw]';

    % Read out and store nested data of vehicle state filtered by
    % current vehicle id via logical table operation.
    
    DataByVehicle.(currentVehicle).state.create_stamp = timestampConversion([HeaderState.create_stamp(currentRowsState).nanoseconds]');
    DataByVehicle.(currentVehicle).state.valid_after_stamp = timestampConversion([HeaderState.valid_after_stamp(currentRowsState).nanoseconds]');
    DataByVehicle.(currentVehicle).state.x = [VehicleStateTable.pose(currentRowsState).x]';
    DataByVehicle.(currentVehicle).state.y = [VehicleStateTable.pose(currentRowsState).y]';
    DataByVehicle.(currentVehicle).state.yaw = [VehicleStateTable.pose(currentRowsState).yaw]';
    
    % Read out and store directly accessible data of vehicle state filtered by
    % current vehicle id via logical table operation.
    for jFields = 4:width(VehicleStateTable)
        currentField = VehicleStateTable.Properties.VariableNames{jFields};
        DataByVehicle.(currentVehicle).state.(currentField) = VehicleStateTable.(currentField)(currentRowsState);
    end
   
end

save ('dds_record', 'DataByVehicle')

end