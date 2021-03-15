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

% This script is intended to be used by every single Matlab HLC script; it loads all required files and sets up all required writers/reader
% WARNING: The state reader waitset does not get initialized here, in case you want to do something else
function [matlabParticipant, stateReader, trajectoryWriter, pathTrackingWriter, systemTriggerReader, readyStatusWriter, trigger_stop] = init_script(matlab_domain_id)
    clc
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    previous_folder = cd(script_directoy); % Remember folder of calling function

    % Import IDL files from cpm library
    dds_idl_matlab = fullfile('../../../cpm_lib/dds_idl_matlab/');
    assert(isfolder(dds_idl_matlab),...
        'Missing directory "%s".', dds_idl_matlab);
    assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
        'No MATLAB IDL-files found in %s', dds_idl_matlab);
    addpath(dds_idl_matlab)

    % XML files for quality of service settings
    middleware_local_qos_xml = '../../../middleware/build/QOS_LOCAL_COMMUNICATION.xml';
    assert(isfile(middleware_local_qos_xml),...
        'Missing middleware local QOS XML "%s"', middleware_local_qos_xml);
    
    ready_trigger_qos_xml = './QOS_READY_TRIGGER.xml';
    assert(isfile(ready_trigger_qos_xml),...
        'Missing ready trigger QOS XML "%s"', ready_trigger_qos_xml);
    
    setenv("NDDS_QOS_PROFILES", ['file://' ready_trigger_qos_xml ';file://' middleware_local_qos_xml]);
    
    %% variables for the communication
    matlabStateTopicName = 'vehicleStateList';
    matlabCommandTopicName = 'vehicleCommandTrajectory';
    matlabCommandTrajectoryTopicName = 'vehicleCommandTrajectory';
    matlabCommandPathTrackingTopicName = 'vehicleCommandPathTracking';
    systemTriggerTopicName = 'systemTrigger';
    readyStatusTopicName = 'readyStatus';
    trigger_stop = uint64(18446744073709551615);

    %% create participants
    matlabParticipant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlab_domain_id);

    %% create reader and writer
    stateReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'VehicleStateList', matlabStateTopicName);
    trajectoryWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandTrajectory', matlabCommandTrajectoryTopicName);
    pathTrackingWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandPathTracking', matlabCommandPathTrackingTopicName);
    systemTriggerReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
    readyStatusWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

    % Get back to folder of calling function
    cd(previous_folder);
end