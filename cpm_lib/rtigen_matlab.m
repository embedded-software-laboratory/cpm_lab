function rtigen_matlab
    dds_idl_matlab = 'dds_idl_matlab';

    % IDL files from cpm library
    cpm_idl_directory = fullfile('../cpm_base/dds_idl');
    if ~exist(cpm_idl_directory, 'dir')
        error(['Missing directory "' cpm_idl_directory '"']);
    end

    % import idl files
    idl_files = [dir(fullfile(cpm_idl_directory, '*.idl'))];
    mkdir(dds_idl_matlab);
    cd(dds_idl_matlab);
    for f = {idl_files.name}
        DDS.import(f{1},'matlab', 'f')
    end
    cd('..')
end