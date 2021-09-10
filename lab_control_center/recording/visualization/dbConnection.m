% dbConnection accesses SQLite database storing rti DDS recording and 
% queries {JSON} formatted sample of vehicle state and vehicle pose.

% todo filtern nach valid stamp beim auslesen der db (WHERE)
function ddsJsonSample = ...
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
getVehicleCommandPathTrackingJsonSample = ['SELECT rti_json_sample FROM "vehicleCommandPathTracking@', ...
                            num2str(dds_domain), ...
                            '"'];

% Apply SQLite queries to database and fetch corresponding data.
ddsJsonSample = struct;
ddsJsonSample.VehicleState = robustFetch(conn,getVehicleStateJsonSample);
ddsJsonSample.VehiclePose = robustFetch(conn,getVehiclePoseJsonSample);
ddsJsonSample.VehicleCommandPathTracking = robustFetch(conn,getVehicleCommandPathTrackingJsonSample);

end

function result = robustFetch(conn, sqlquery)
try
	result = fetch(conn, sqlquery);
catch
	result = [];
end
end
