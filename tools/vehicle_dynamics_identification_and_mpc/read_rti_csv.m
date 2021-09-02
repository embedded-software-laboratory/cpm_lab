function result = read_rti_csv(filepath)

    fID = fopen(filepath, 'r');
    
    line1 = fgetl(fID);
    line2 = fgetl(fID);
    
    fclose(fID);
    
    column_header = strsplit(line2,',');
    
    for i = 1:length(column_header)
        column_header{i} = strrep(column_header{i},'.','_');
    end
    
    data = dlmread(filepath, ',',2,0);
    
    assert(size(data,2) == length(column_header));
    
    result = struct;    
    
    for i = 1:length(column_header)
        result.(column_header{i}) = data(:,i);
    end

end

