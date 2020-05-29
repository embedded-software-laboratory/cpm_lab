vehicle_id = 1;
dds_domain = 99;
recording_folders = dir('/tmp/rti_recordings/*');
current_folder = recording_folders(end);
assert(current_folder.isdir);
current_recording = fullfile(...
    current_folder.folder, ...
    current_folder.name, ...
    'recording.dat' ...
);
overviewPlot(vehicle_id, dds_domain, current_recording);