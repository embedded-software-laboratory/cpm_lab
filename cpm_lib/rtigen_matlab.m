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