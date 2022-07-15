% This script is intended to be used by every single Matlab HLC script; it loads all required files and sets up all required writers/reader
% WARNING: The state reader waitset does not get initialized here, in case you want to do something else
% CAVE `matlabParticipant`must be stored for RTI DDS somewhere in the workspace  (so it doesn't get gc'ed)
function [matlabParticipant, stateReader, trajectoryWriter,...
    pathTrackingWriter, systemTriggerReader, readyStatusWriter,...
    trigger_stop, directWriter] = init_script(matlab_domain_id)
    % Inputs
    %   matlab_domain_id (int):         domain ID to be used for DDS

    script_directory = fileparts([mfilename('fullpath') '.m']);
    previous_folder = cd(script_directory); % Remember folder of calling function

    % Import IDL files from cpm library
    dds_idl_matlab = fullfile('../../../cpm_lib/dds_idl_matlab/');
    assert(isfolder(dds_idl_matlab),...
        'Missing directory "%s".', dds_idl_matlab);
    assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
        'No MATLAB IDL-files found in %s', dds_idl_matlab);
    addpath(dds_idl_matlab)

    % XML files for quality of service settings
    middleware_local_qos_xml = fullfile(...
                    script_directory, ...
                    '../../../middleware/build/QOS_LOCAL_COMMUNICATION.xml');
    assert(isfile(middleware_local_qos_xml),...
        'Missing middleware local QOS XML "%s"', middleware_local_qos_xml);
    
    ready_trigger_qos_xml = fullfile(...
                    script_directory,...
                    './QOS_READY_TRIGGER.xml');
    assert(isfile(ready_trigger_qos_xml),...
        'Missing ready trigger QOS XML "%s"', ready_trigger_qos_xml);
    
    qos_env_var = "NDDS_QOS_PROFILES";
    qos_profiles = getenv( qos_env_var );
    % Quick and dirty check wether these profiles are already part of the environment variable
    if ~( contains( qos_profiles, middleware_local_qos_xml )...
        || contains( qos_profiles, ready_trigger_qos_xml ) )
        qos_profiles = [qos_profiles, ';file://' , ready_trigger_qos_xml , ';file://' , middleware_local_qos_xml];
        % If qos_profiles begins with a semicolon now, removed the semicolon
        if qos_profiles(1) == ';'
            qos_profiles = qos_profiles(2:end);
        end
        setenv( qos_env_var , qos_profiles );
    end
    
    %% variables for the communication
    matlabStateTopicName = 'vehicleStateList';
    matlabCommandTrajectoryTopicName = 'vehicleCommandTrajectory';
    matlabCommandPathTrackingTopicName = 'vehicleCommandPathTracking';
    matlabCommandDirectTopicName = 'vehicleCommandDirect';
    systemTriggerTopicName = 'systemTrigger';
    readyStatusTopicName = 'readyStatus';
    trigger_stop = uint64(18446744073709551615);

    %% create participants
    matlabParticipant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlab_domain_id);

    %% create reader and writer
    stateReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'VehicleStateList', matlabStateTopicName);
    trajectoryWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandTrajectory', matlabCommandTrajectoryTopicName);
    pathTrackingWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandPathTracking', matlabCommandPathTrackingTopicName);
    directWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'VehicleCommandDirect', matlabCommandDirectTopicName);
    systemTriggerReader = DDS.DataReader(DDS.Subscriber(matlabParticipant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
    readyStatusWriter = DDS.DataWriter(DDS.Publisher(matlabParticipant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

    % Get back to folder of calling function
    cd(previous_folder);
end
