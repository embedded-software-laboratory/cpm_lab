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

% Apply SQLite queries to database and fetch corresponding data.
ddsJsonSample = struct;
ddsJsonSample.VehicleState = fetch(conn,getVehicleStateJsonSample);
ddsJsonSample.VehiclePose = fetch(conn,getVehiclePoseJsonSample);

save ('dds_json_sample', 'ddsJsonSample')

end
