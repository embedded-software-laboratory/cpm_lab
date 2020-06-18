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

function optimize_parameters_delay_grid
    for n_delay_steps_IPS = 0:1
        for n_delay_steps_local = 0:1
            for n_delay_steps_steering = 2:8
                for n_delay_steps_motor = 0:8
                    
                    file_name = sprintf('output/optimal_parameters_d_ips_%i_d_local_%i_d_steer_%i_d_mot_%i.mat', ...
                        n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor);
                    
                    
                    if exist(file_name, 'file')
                        fprintf('Skipping "%s"\n', file_name);
                    else
                        optimize_parameters(file_name, n_delay_steps_IPS, n_delay_steps_local, n_delay_steps_steering, n_delay_steps_motor);
                    end
                    
                end
            end
        end
    end
end