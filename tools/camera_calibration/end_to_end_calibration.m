function end_to_end_calibration
    
    clc
    D = dlmread('calibration_points.csv',';',1,0);
    wx = D(:,1);
    wy = D(:,2);
    ix = D(:,3) / 2048;
    iy = D(:,4) / 2048;
    
    feature_matrix = [...
        (ix*0+1) ...
        (ix) (iy) ...
        (ix.^2) (ix.*iy) (iy.^2) ...
        (ix.^3) (ix.^2 .* iy) (ix .* iy.^2) (iy.^3) ...
        (ix.^4) (ix.^3 .* iy) (ix.^2 .* iy.^2) (ix .* iy.^3) (iy.^4) ];
    
    calibration_x = feature_matrix \ wx;
    calibration_y = feature_matrix \ wy;
    
    
    format compact
    reprojection_error_x = max(abs(feature_matrix * calibration_x - wx))
    reprojection_error_y = max(abs(feature_matrix * calibration_y - wy))
    
    
    
    
    fprintf('calibration_x: \n');
    fprintf('%e, ', calibration_x);
    fprintf('\n');
    
    fprintf('calibration_y: \n');
    fprintf('%e, ', calibration_y);
    fprintf('\n');
    
    
    %% Result
    % reprojection_error_x =
    %     0.0214
    % reprojection_error_y =
    %     0.0197
    % calibration_x: 
    % 4.641747e+00, -5.379232e+00, -3.469735e-01, 1.598328e+00, 9.661605e-01, 3.870296e-01, -1.125387e+00, -1.264416e-01, -9.323793e-01, 5.223107e-02, 5.771384e-02, 7.367979e-02, 5.512993e-02, 3.857936e-02, -2.401879e-02, 
    % calibration_y: 
    % -5.985142e-01, 6.235073e-01, 5.412047e+00, -6.668763e-01, -1.038656e+00, -1.593830e+00, 2.284258e-01, 1.090997e+00, 9.839002e-02, 1.065165e+00, -8.319799e-02, -8.929741e-02, -5.524760e-02, -2.687517e-02, -5.680051e-02, 

    
end

