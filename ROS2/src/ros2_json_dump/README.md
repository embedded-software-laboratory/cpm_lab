Use the bash command

    ros2 run ros2_json_dump dump | nc -lk localhost 1234

to make all ROS2 messages available through a TCP connection.


MATLAB snippet for receiveing the data:

    function main
        clc
        tcp = tcpip('127.0.0.1', 1234);
        tcp.InputBufferSize = 2^15;
        fopen(tcp);
        finishup = onCleanup(@()fclose(tcp));
        while true
            packets = read_new_packets(tcp);
            if numel(packets) > 0
                fprintf('%f\n',packets.receive_stamp/1e9);
            end
            pause(0.01);
        end
    end
    function packets = read_new_packets(tcp) 
        packets = struct([]);
        while tcp.BytesAvailable > 0
            data = fread(tcp,tcp.BytesAvailable);
            data = char(data)';
            obj = [];
            try
                obj = jsondecode(data);
            end
            if isstruct(obj) && isfield(obj, 'topic')
                packets = [packets obj];
            end        
        end 
    end

