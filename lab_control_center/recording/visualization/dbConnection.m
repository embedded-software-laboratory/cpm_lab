% dbConnection accesses SQLite database storing rti DDS recording and 
% queries {JSON} formatted sample of vehicle state and vehicle pose.

% todo filtern nach valid stamp beim auslesen der db (WHERE)
function [ddsVehicleStateJsonSample, ddsVehiclePoseJsonSample] = ...
    dbConnection(dds_domain, recording_file)

assert(isfile(recording_file));
conn = sqlite(recording_file);

% Specify data to be selected from database by SQLite queries.
getVehicleStateJsonSample = ['SELECT rti_json_sample FROM "vehicleState@', ...
                             num2str(dds_domain), ...
                             '"'];
getVehiclePoseJsonSample = ['SELECT rti_json_sample FROM "vehicleObservation@', ...
                            num2str(dds_domain), ...
                            '"'];

% Apply SQLite queries to database and fetch corresponding data.
ddsVehicleStateJsonSample = fetch(conn,getVehicleStateJsonSample);
ddsVehiclePoseJsonSample = fetch(conn,getVehiclePoseJsonSample);

save ('dds_json_sample', 'ddsVehicleStateJsonSample', 'ddsVehiclePoseJsonSample')

end