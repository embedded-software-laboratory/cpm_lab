function rtigen_matlab
    setenv("LD_LIBRARY_PATH", ['/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0']);
    dds_idl_matlab_dir = 'dds_idl_matlab';

    % IDL files from cpm library
    dds_idl_dir = fullfile('dds_idl');
    if ~exist(dds_idl_dir, 'dir')
        error(['Missing directory "' dds_idl_dir '"']);
    end
    addpath(dds_idl_dir)

    % import idl files
    idl_files = [dir(fullfile(dds_idl_dir, '*.idl'))];
    mkdir(dds_idl_matlab_dir);
    cd(dds_idl_matlab_dir);
    for f = {idl_files.name}
        DDS.import(f{1},'matlab', 'f')
    end
end