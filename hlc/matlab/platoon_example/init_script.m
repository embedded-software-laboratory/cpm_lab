% This script is intended to be used by every single Matlab HLC script; it loads all required files and sets up all required writers/reader
% WARNING: The state reader waitset does not get initialized here, in case you want to do something else
function [matlabParticipant, stateReader, trajectoryWriter, systemTriggerReader, readyStatusWriter, trigger_stop] = init_skript(matlab_domain_id)
    clc
    script_directoy = fileparts([mfilename('fullpath') '.m']);
    previous_folder = cd(script_directoy); % Remember folder of calling function

    % IDL files from cpm library
    dds_idl_matlab = fullfile('../../../../cpm_base/cpm_lib/dds_idl_matlab/');
    if ~exist(dds_idl_matlab, 'dir')
        error(['Missing directory "' dds_idl_matlab '"']);
    end
    addpath(dds_idl_matlab)

    % XML files for quality of service settings
    middleware_local_qos_xml = ['../../middleware/build/QOS_LOCAL_COMMUNICATION.xml'];
    if ~exist(middleware_local_qos_xml,'file')
        error(['Missing middleware local QOS XML "' middleware_local_qos_xml '"'])
    end

    ready_trigger_qos_xml = ['./QOS_READY_TRIGGER.xml'];
    if ~exist(ready_trigger_qos_xml,'file')
        error(['Missing ready trigger QOS XML "' ready_trigger_qos_xml '"'])
    end
    
    setenv("NDDS_QOS_PROFILES", ['file://' ready_trigger_qos_xml ';file://' middleware_local_qos_xml]);
    
    %% variables for the communication
    matlabStateTopicName = 'vehicleStateList';
    matlabCommandTopicName = 'vehicleCommandTrajectory';
    systemTriggerTopicName = 'systemTrigger';
    readyStatusTopicName = 'readyStatus';
    trigger_stop = uint64(18446744073709551615);

    phaseTime = 40;

    %% create participants
    matlabParticipant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlab_domain_id);

    %% create reader and writer
    stateReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'VehicleStateList', matlabStateTopicName);
    trajectoryWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandTrajectory', matlabCommandTopicName);
    systemTriggerReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
    readyStatusWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

    % Get back to folder of calling function
    cd(previous_folder);
end