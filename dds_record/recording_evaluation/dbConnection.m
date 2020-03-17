% dbConnection accesses SQLite database storing rti DDS recording and 
% queries {JSON} formatted sample of vehicle state and vehicle pose.

% todo filtern nach valid stamp beim auslesen der db (WHERE)
function [ddsVehicleStateJsonSample, ddsVehiclePoseJsonSample] = dbConnection()

dbFile = '/Users/valeriepfannschmidt/Desktop/CPM/dds_record/recording_evaluation/rti_recorder_default_json.dat';
conn = sqlite(dbFile);

% Specify data to be selected from database by SQLite queries.
getVehicleStateJsonSample = 'SELECT rti_json_sample FROM "vehicleState@77"';
getVehiclePoseJsonSample = 'SELECT rti_json_sample FROM "vehicleObservation@77"';

% Apply SQLite queries to database and fetch corresponding data.
ddsVehicleStateJsonSample = fetch(conn,getVehicleStateJsonSample);
ddsVehiclePoseJsonSample = fetch(conn,getVehiclePoseJsonSample);

save ('dds_json_sample', 'ddsVehicleStateJsonSample', 'ddsVehiclePoseJsonSample')

end