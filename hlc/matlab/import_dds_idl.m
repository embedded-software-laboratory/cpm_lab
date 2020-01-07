function import_dds_idl
    dds_idl_matlab = 'dds_idl_matlab';
    if ~exist(dds_idl_matlab, 'dir')
        % IDL files from cpm library
        cpm_idl_directory = fullfile('../../../cpm_base/dds_idl');
        if ~exist(cpm_idl_directory, 'dir')
            error(['Missing directory "' cpm_idl_directory '"']);
        end
        addpath(cpm_idl_directory);
        % IDL files from hlc
        hlc_idl_directory = fullfile('../middleware/idl');
        if ~exist(hlc_idl_directory, 'dir')
            error(['Missing directory "' hlc_idl_directory '"']);
        end
        addpath(hlc_idl_directory);
        % import idl files
        idl_files = [dir(fullfile(cpm_idl_directory, '*.idl')); ...
                     dir(fullfile(hlc_idl_directory, '*.idl'))];
        mkdir(dds_idl_matlab);
        cd(dds_idl_matlab);
        for f = {idl_files.name}
            DDS.import(f{1},'matlab', 'f')
        end
        cd('..')
    end
    addpath(dds_idl_matlab);
end