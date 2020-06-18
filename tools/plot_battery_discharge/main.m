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

    data = [ ... % Time(minutes), voltage
                    0         8.25   ; ...
                    18         8.05   ; ...
                    40         7.83   ; ...
                    60        7.68    ; ...
                    80        7.55    ; ...
                    101       7.47   ; ...
                    120       7.42   ; ...
                    140       7.35   ; ...
                    150       7.27   ; ...
                    160       7.22   ; ...
                    164       7.0   ; ...
                    167       6.8   ; ...
                    168       6.5   ; ...
                    169       6.4    ...
    ];
    

    clf
    box on
    plot(data(:,1), data(:,2))
    grid on
    xlabel('t [min]','FontSize',20)
    ylabel('Volt','FontSize',20)
    print('discharge_curve.png','-dpng')

    
end

