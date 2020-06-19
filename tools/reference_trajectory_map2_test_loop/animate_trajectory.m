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

function animate_trajectory
        
    reference_path = dlmread('reference_path.csv');
    path_x = reference_path(:,1);
    path_y = reference_path(:,2);
    
    opt_result = load('optimization_checkpoint.mat');
    
    clf
    hold on
    plot(path_x, path_y);
    axis equal
    h = plot(0,0);
    plot(4.5*[0 0 1 1 0],4*[0 1 1 0 0],'k','LineWidth',4)

    position_fn = full(opt_result.s);
    
    k = 1;
    while true
        vehicle_x = [];
        vehicle_y = [];
        
        circle_c = [0.1*cos(linspace(0,2*pi,100)) nan];
        circle_s = [0.1*sin(linspace(0,2*pi,100)) nan];
        
        for i = 1:opt_result.p.nVeh
            
            
            t_idx = opt_result.p.delta_H_veh * i + k;
            s = circshift(position_fn, t_idx, 1);
            s_idx = s(1) / opt_result.p.ds + 1;
            s_idx = round(s_idx);
            
            tmp_x = circshift(path_x, s_idx, 1);
            tmp_y = circshift(path_y, s_idx, 1);
            
            vehicle_x = [vehicle_x (circle_c+tmp_x(1))];
            vehicle_y = [vehicle_y (circle_s+tmp_y(1))];
        end
        
        set(h, 'XData', vehicle_x);
        set(h, 'YData', vehicle_y);
        drawnow
        k = k + 2;
    end
    
end

