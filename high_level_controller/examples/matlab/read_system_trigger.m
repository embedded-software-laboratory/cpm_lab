function [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop)
%READ_SYSTEM_TRIGGER Reads system trigger topic for start and stop signals
%   [GOT_START, GOT_STOP] = READ_SYSTEM_TRIGGER(READER_SYSTEMTRIGGER, TRIGGER_STOP)
%   where READER_SYSTEMTRIGGER is a DDS reader of the systemTrigger topic and
%   TRIGGER_STOP is the agreed number for a stop signal,
%   returns true as output if the corresponding signal was received,
%   false if it was not received.
    [trigger, ~, sample_count, ~] = reader_systemTrigger.take();
    got_stop = false;
    got_start = false;
    if sample_count > 0
        % look at most recent signal with (end)
        if trigger(end).next_start().nanoseconds() == trigger_stop
            got_stop = true;
        else
            got_start = true;
        end
    end
end