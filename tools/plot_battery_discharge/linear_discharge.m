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


data = [ ... % Time(minutes), voltage
                  0       8.17   ; ... 
                 20       8.05   ; ...
                 40       7.97   ; ...
                 60       7.82   ; ...
                 80       7.72   ; ... 
                101       7.64  ; ... 
                120       7.56   ; ...
                140       7.48   ; ...
                150       7.46  ; ...
                160       7.42   ; ... 
                170       7.41    ; ... 
                180        7.39   ; ...
                190         7.37  ; ...
                200         7.36  ; ...
                210         7.36    ; ...
                220         7.35 ; ...
                230         7.35 ; ...
                240         7.35 ; ...
                260         7.32 ; ...
                280         7.27 ; ...
                300         7.18 ; ...
                310         7.07 ; ...
                320         6.93 ; ...
                323         6.88 ; ...
                326         6.85 ; ...
                329         6.80 ; ...
                332         6.72 ; ...
                335         6.62 ; ...
                337         6.53 ; ...
                339         6.45 ; ...
                340        6.37 ; ...
                341          6.29 ; ...
                342         6.28
                
];

l1_t1 = 0;
l1_t2 = 140;
l1_v1 = 8.17;
l1_v2 = 7.48;
l1_t = l1_t1:l1_t2;
l1_v = (l1_v2-l1_v1) / (l1_t2-l1_t1) * (l1_t-l1_t1) + l1_v1;

l5_t1 = 140;
l5_t2 = 190;
l5_v1 = 7.48;
l5_v2 = 7.37;
l5_t = l5_t1:l5_t2;
l5_v = (l5_v2-l5_v1) / (l5_t2-l5_t1) * (l5_t-l5_t1) + l5_v1;

l2_t1 = 190;
l2_t2 = 240;
l2_v1 = 7.37;
l2_v2 = 7.35;
l2_t = l2_t1:l2_t2;
l2_v = (l2_v2-l2_v1) / (l2_t2-l2_t1) * (l2_t-l2_t1) + l2_v1;


l3_t1 = 240;
l3_t2 = 300;
l3_v1 = 7.35;
l3_v2 = 7.17;
l3_t = l3_t1:l3_t2;
l3_v = (l3_v2-l3_v1) / (l3_t2-l3_t1) * (l3_t-l3_t1) + l3_v1;


l4_t1 = 300;
l4_t2 = 329;
l4_v1 = 7.16;
l4_v2 = 6.8;
l4_t = l4_t1:l4_t2;
l4_v = (l4_v2-l4_v1) / (l4_t2-l4_t1) * (l4_t-l4_t1) + l4_v1;

l6_t1 = 329;
l6_t2 = 340;
l6_v1 = 6.8;
l6_v2 = 6.3;
l6_t = l6_t1:l6_t2;
l6_v = (l6_v2-l6_v1) / (l6_t2-l6_t1) * (l6_t-l6_t1) + l6_v1;





figure(1)
clf
box on
plot(data(:,1), data(:,2))
grid on
xlabel('t [min]','FontSize',20)
ylabel('Volt','FontSize',20)

hold on
plot(l1_t, l1_v)
plot(l5_t, l5_v)
plot(l2_t, l2_v)
plot(l3_t, l3_v)
plot(l4_t, l4_v)
plot(l6_t, l6_v)
% hold off

figure(2)
clf
box on
max_time = max(data(:,1));
plot(data(:,2), abs(data(:,1)-max_time)/max_time*100)
grid on
xlabel('Volt','FontSize',20)
ylabel('Batterieladung [%]','FontSize',20)

% hold on
% plot(l1_v, l1_battery(l1_v))
% plot(l2_v, l2_battery(l2_v))
% plot(l3_v, l3_battery(l3_v))
% hold off