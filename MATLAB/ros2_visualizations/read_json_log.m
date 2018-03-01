function messages = read_json_log(filename)

    id = fopen(filename);
    data = fread(id);
    fclose(id);

    messages = struct([]);
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