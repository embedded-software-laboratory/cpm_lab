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

% denesting resolves the nested input structs created by decoding the
% JSON-formatted samples of rti DDS recordings by function dbConnection()  
% to allow ready accessibilty when plotting.
% It creates the struct DataByVehicle with first level fields
% distinguishing between the vehicles, seconds level fields 'observation' 
% and 'state' corresponding to the topic of rti DDS the data were recorded 
% from and third level fields referring to the data measured. Labels from
% original recording are kept as fieldnames.

function [DataByVehicle] = preprocessing(dds_domain, recording_file)
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

%% Find timepoint zero, i.e. smallest timestamp in recording
t_start_state_nanos = min([HeaderState.create_stamp.nanoseconds]);
t_start_observation_nanos = min([HeaderObservation.create_stamp.nanoseconds]);
t_start_nanos = min(t_start_state_nanos, t_start_observation_nanos);

%% Sort data by vehicle ids, resolve nested structures and store into struct DataByVehicle.

DataByVehicle = struct;

for iVehicles = 1:max([VehicleStateRaw.vehicle_id]) % Loop over vehicle ids.

    % Build logical array of all rows in data tables corresponding to current vehicle.
    currentRowsState = VehicleStateTable.vehicle_id == iVehicles;
    currentRowsObservation = VehicleObservationTable.vehicle_id == iVehicles; 

    currentVehicle = ['vehicle_' int2str(iVehicles)]; % Create fieldname from current vehicle id.

    % Read out and store nested data of vehicle observation filtered by
    % current vehicle id via logical table operation.

    DataByVehicle.(currentVehicle).observation.create_stamp_nanos = [HeaderObservation.create_stamp(currentRowsObservation).nanoseconds]' - t_start_nanos;
    DataByVehicle.(currentVehicle).observation.valid_after_stamp_nanos = [HeaderObservation.valid_after_stamp(currentRowsObservation).nanoseconds]' - t_start_nanos;
    DataByVehicle.(currentVehicle).observation.create_stamp = 1e-9 * DataByVehicle.(currentVehicle).observation.create_stamp_nanos;
    DataByVehicle.(currentVehicle).observation.valid_after_stamp = 1e-9 * DataByVehicle.(currentVehicle).observation.valid_after_stamp_nanos;
    DataByVehicle.(currentVehicle).observation.x = [VehicleObservationTable.pose(currentRowsObservation).x]';
    DataByVehicle.(currentVehicle).observation.y = [VehicleObservationTable.pose(currentRowsObservation).y]';
    DataByVehicle.(currentVehicle).observation.yaw = [VehicleObservationTable.pose(currentRowsObservation).yaw]';

    % Read out and store nested data of vehicle state filtered by
    % current vehicle id via logical table operation.
    DataByVehicle.(currentVehicle).state.create_stamp_nanos = [HeaderState.create_stamp(currentRowsState).nanoseconds]' - t_start_nanos;
    DataByVehicle.(currentVehicle).state.valid_after_stamp_nanos = [HeaderState.valid_after_stamp(currentRowsState).nanoseconds]' - t_start_nanos;
    DataByVehicle.(currentVehicle).state.create_stamp = 1e-9 * DataByVehicle.(currentVehicle).state.create_stamp_nanos;
    DataByVehicle.(currentVehicle).state.valid_after_stamp = 1e-9 * DataByVehicle.(currentVehicle).state.valid_after_stamp_nanos;
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
