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
    reprojection_error_x = median(abs(feature_matrix * calibration_x - wx))
    reprojection_error_y = median(abs(feature_matrix * calibration_y - wy))
    
    
    
    
    fprintf('calibration_x: \n');
    fprintf('%e, ', calibration_x);
    fprintf('\n');
    
    fprintf('calibration_y: \n');
    fprintf('%e, ', calibration_y);
    fprintf('\n');
    
    
    %% Result
    % reprojection_error_x =
    % 0.0026
    % reprojection_error_y =
    % 0.0026
    % calibration_x: 
    % 4.351151e+00, -4.863260e+00, -1.183271e-01, 1.094383e+00, 4.861627e-01, 1.418471e-01, -6.700942e-01, 3.049470e-01, -4.362105e-01, 1.852765e-01, -7.676803e-02, -7.289893e-02, -1.313038e-01, -8.945004e-02, -5.608838e-02, 
    % calibration_y: 
    % -3.409322e-01, 5.226075e-01, 4.909991e+00, -3.901540e-01, -5.209485e-01, -1.093096e+00, 6.016393e-02, 5.101132e-01, -3.097603e-01, 5.542172e-01, -2.875253e-02, 5.073364e-02, 1.783163e-01, 1.005458e-01, 1.114947e-01, 

    
end

