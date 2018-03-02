function messages = read_json_tcp(tcp)

    messages = struct([]);
    if tcp.BytesAvailable > 0
        data = fread(tcp,tcp.BytesAvailable);
        text = char(data');

        lines = splitlines(text);

        for line = lines'
            line_str = line{1};
            try
                m = jsondecode(line_str);
                messages = [messages m];
            end
        end
    end
end