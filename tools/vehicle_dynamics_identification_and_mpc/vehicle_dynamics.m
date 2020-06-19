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

function dx = vehicle_dynamics(x,u,p)

    %% See 'vehicle_dynamics.png/tex' for documentation.
    % Cross check with vehicle paper
    
    px    = x(:,1);
    py    = x(:,2);
    yaw   = x(:,3);
    v     = x(:,4);
    
    f          = u(:,1);
    delta_ref  = u(:,2);
    
    delta = delta_ref + p(8);
    
    dx = 0*x;
    
    dx(:,1) = p(1) .* v .* (1 + p(2) .* delta.^2) .* cos(yaw + p(3) .* delta + p(9));
    dx(:,2) = p(1) .* v .* (1 + p(2) .* delta.^2) .* sin(yaw + p(3) .* delta + p(9));
    dx(:,3) = p(4) .* v .* delta;
    dx(:,4) = p(5) .* v + p(6) .* sign(f) .* (abs(f).^(p(7)));
    
end

