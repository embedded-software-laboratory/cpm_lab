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

function [x,w] = quadrature_LGL(N)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % lglnodes.m
    %
    % Computes the Legendre-Gauss-Lobatto nodes, weights and the LGL Vandermonde 
    % matrix. The LGL nodes are the zeros of (1-x^2)*P'_N(x). Useful for numerical
    % integration and spectral methods. 
    %
    % Reference on LGL nodes and weights: 
    %   C. Canuto, M. Y. Hussaini, A. Quarteroni, T. A. Tang, "Spectral Methods
    %   in Fluid Dynamics," Section 2.3. Springer-Verlag 1987
    %
    % Written by Greg von Winckel - 04/17/2004
    % Contact: gregvw@chtm.unm.edu
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Truncation + 1
    N1=N+1;
    digits(max([digits() 70]))

    % Use the Chebyshev-Gauss-Lobatto nodes as the first guess
    x=vpa(cos(pi*(0:N)/N)');

    % The Legendre Vandermonde Matrix
    P=zeros(N1,N1);

    % Compute P_(N) using the recursion relation
    % Compute its first and second derivatives and 
    % update x using the Newton-Raphson method.

    xold=2;

    my_eps = vpa(10^(-digits()+10));
    while max(abs(x-xold))>my_eps
        xold=vpa(x);        
        P(:,1)=1;
        P(:,2)=x;
        for k=2:N
            P(:,k+1)=vpa(( (2*k-1)*x.*P(:,k)-(k-1)*P(:,k-1) )/k);
        end
        x=vpa(xold-( x.*P(:,N1)-P(:,N) )./( N1*P(:,N1) ));
        P = vpa(P);
    end


    w=2./(N*N1*P(:,N1).^2);
    w = flipud(w);
    x = flipud(x);

end