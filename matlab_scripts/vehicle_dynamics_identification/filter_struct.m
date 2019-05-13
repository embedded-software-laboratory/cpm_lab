function strct = filter_struct(strct, filter)
    
    fname = fieldnames(strct);
    
    for i = 1:length(fname)
        data = strct.(fname{i});
        strct.(fname{i}) = data(filter);
    end
    
end

