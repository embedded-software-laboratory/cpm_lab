function main_dummy_loop(varargin)
    % Initialize data readers/writers...
    init_script_path = fullfile( ...
        getenv('HOME'), 'dev/software/high_level_controller/examples', ...
        'matlab/init_script.m' ...
    );
    assert(isfile(init_script_path), 'Missing file "%s".', init_script_path);
    addpath(fileparts(init_script_path));    matlabDomainId = 1;
    [~, ~, ~, reader_systemTrigger, ~, trigger_stop] = init_script(matlabDomainId);
    got_stop = false;
    while (~got_stop)
        pause(0.5);
        % Check for stop signal
        [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    end
end