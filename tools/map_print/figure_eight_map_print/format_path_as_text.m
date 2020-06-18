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

function format_path_as_text(path)

clc

% trim to make a perfect loop
dist = sqrt((path(:,1)-path(end,1)).^2 + (path(:,2)-path(end,2)).^2);
i=find(diff(dist)>0);
i=i(1)+1;
path = path(i:end,:);

% output controller path
f = fopen('path.h','w');

fprintf(f,'const vector<double> path_x {');
fprintf(f,'%f,',path(:,1)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_y {');
fprintf(f,'%f,',path(:,2)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_yaw {');
fprintf(f,'%f,',path(:,3)');
fprintf(f,'};\n');

fprintf(f,'const vector<double> path_curvature {');
fprintf(f,'%f,',path(:,4)');
fprintf(f,'};\n');

fclose(f);


% output matlab visualization path
f = fopen('path.m','w');

fprintf(f,'    path_x = [');
fprintf(f,'%f,',path(1:50:end,1)');
fprintf(f,'];\n');

fprintf(f,'    path_y = [');
fprintf(f,'%f,',path(1:50:end,2)');
fprintf(f,'];\n');

fclose(f);


end

