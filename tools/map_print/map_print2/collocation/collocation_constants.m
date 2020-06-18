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

function [nodes, w_integration, D, w_interp] = collocation_constants(node_name, N, a, b, as_sym)


    
    if strcmp(node_name, 'LGL')
        [nodes, w_integration] = quadrature_LGL(N);
    elseif strcmp(node_name, 'CGL')
        [nodes, w_integration] = quadrature_CGL(N);
    end
    
    assert(all(diff(double(nodes))>0));
    %assert(abs(sum(double(w_integration)) - 2) < 1e-14);

    
    nodes = 0.5*(b-a)*(nodes+1) + a;
    w_integration = w_integration * 0.5 * (b-a);

    w_interp = Lagrange_interpolation_weights(nodes);
    D = Lagrange_differentiation_matrix(nodes, w_interp);
    
    if nargin ~= 5 || ~as_sym
        w_integration = double(w_integration);
        nodes = double(nodes);
        D = double(D);
        w_interp = double(w_interp);
    end

end