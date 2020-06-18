% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function pose_calibration

    %% Load measurement data
    % This data is taken from "indoor_positioning_system/src/PoseCalculation.cpp"
    % with std::cout << features[i] ...
    data = dlmread('pose_calibration.txt',',');
    data = data(:,1:5);
    
    %% Split data into continuous sequences and take average of each sequence
    I = find(vecnorm(diff(data)') > 0.4);
    I = [1 I size(data,1)];
    sequence_mean = zeros(0,5);
    for i = 1:(length(I)-1)
        slice = (I(i)+1):(I(i+1));
        if length(slice) > 5
            sequence = data(slice,:);
            sequence_mean(end+1,:) = mean(sequence);
            fprintf('sequence variance: %e\n', norm(var(sequence)));
            if norm(var(sequence)) > 0.1
                warning('large variance');
            end
        end
    end
    
    %% Generate expected calibration sequence
    expected_position_x = [];
    expected_position_y = [];
    expected_direction_x = [];
    expected_direction_y = [];
    for y = [0.6, 3.4]
        for x = [0.6, 3.9]
            for a = (0:7)                
                expected_position_x(1,end+1) = x;
                expected_position_y(1,end+1) = y;
                expected_direction_x(1,end+1) = cos(a * (45/180*pi));
                expected_direction_y(1,end+1) = sin(a * (45/180*pi));
            end
        end
    end
    
    %% Correct number of measurements?
    assert(length(expected_position_x) == size(sequence_mean, 1));
    
    calibration_position_x = sequence_mean \ expected_position_x';
    calibration_position_y = sequence_mean \ expected_position_y';
    calibration_direction_x = sequence_mean \ expected_direction_x';
    calibration_direction_y = sequence_mean \ expected_direction_y';
    
    calibration_matrix = [calibration_position_x calibration_position_y calibration_direction_x calibration_direction_y]';
    
    
    reprojection_error_position_x = max(abs(sequence_mean * calibration_position_x - expected_position_x'))
    reprojection_error_position_y = max(abs(sequence_mean * calibration_position_y - expected_position_y'))
    reprojection_error_direction_x = max(abs(sequence_mean * calibration_direction_x - expected_direction_x'))
    reprojection_error_direction_y = max(abs(sequence_mean * calibration_direction_y - expected_direction_y'))


    calibration_matrix

end






