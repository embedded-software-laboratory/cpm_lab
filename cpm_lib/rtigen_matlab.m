function rtigen_matlab
    setenv("LD_LIBRARY_PATH", ['/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0']);
    dds_idl_matlab = 'dds_idl_matlab';

    % IDL files from cpm library
    cpm_idl_directory = fullfile('../dds_idl');
    if ~exist(cpm_idl_directory, 'dir')
        error(['Missing directory "' cpm_idl_directory '"']);
    end
    addpath(cpm_idl_directory)

    % import idl files
    idl_files = [dir(fullfile(cpm_idl_directory, '*.idl'))];
    mkdir(dds_idl_matlab);
    cd(dds_idl_matlab);
    for f = {idl_files.name}
        DDS.import(f{1},'matlab', 'f')
    end
    cd('..')
end