function [vehicle_id] = extractVehicleID(idString)

parts = strsplit(idString, '_');
vehicle_id = str2num(parts{2});

end