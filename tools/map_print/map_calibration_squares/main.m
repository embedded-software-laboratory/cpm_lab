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

function main
    

    plot_scale = 1000 / 25.4 * 72; % 1:1 scale
    plot_scale = 50 / 25.4 * 72; % 1:20 scale
    plot_scale = 100 / 25.4 * 72; % 1:10 scale
    
    full_width = 4.5;
    full_height = 4.0;
    
    close all
    figure('Visible','off')
    hold on
    axis equal
    
    fig = gcf;
    fig.Color = 'white';
    
    xlim(plot_scale*[-1 1] * full_width/2)
    ylim(plot_scale*[-1 1] * full_height/2)
    
    for i = -23:23
        for j = -21:21
            if mod(i+j, 2) == 0
                patch(...
                    plot_scale*(0.05*[-1 1 1 -1]+0.0000001+0.1*(i-1)),...
                    plot_scale*(0.05*[-1 -1 1 1]+0.0000001+0.05+0.1*(j-1)),...
                    [0 0 0], 'EdgeColor', 'none')
            end
        end
    end

    axis off
    ax = gca;
    ax.Position = [0 0 1 1];
    fig.PaperPositionMode = 'manual';
    fig.PaperType = '<custom>';
    fig.PaperUnits = 'points';
    fig.PaperSize = [full_width full_height] * plot_scale;
    fig.InvertHardcopy = 'off';
    fig.PaperPosition = [0 0 full_width full_height] * plot_scale;

    print(fig, 'x.pdf', '-dpdf', '-painters')
end

