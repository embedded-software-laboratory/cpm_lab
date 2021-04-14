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

%======================================================================
%> @brief Defines vehicle properties as well properties of the dynamics.
%>
%> More detailed description.
%>
%> @retval model Structure with the model properties:
%>  * ode: Function link to the system dgl function
%>  * nx: Amount of state variables
%>  * nu: Amount of input variables
%>  * ny: Amount of output variables
%======================================================================
function [ model ] = model()
    model = struct;
    model.ode = @ode;
    model.A = [0,1,0; 0,0,1; 0,0,-1/0.1];
    model.B = [0;0;1/0.1];
    model.C = [1,0,0; 0,1,0 ];  % outputs are s and v
    model.nx = 3;
    model.nu = 1;
    model.ny = 2;
end

%======================================================================
%> @brief Defines system differential equations parametically.
%>
%> @param x Current state vector:
%> * x1: s-position
%> * x2: velocity
%> * x3: acceleration
%> @param u_ref Reference input values, which the input values shall have,
%> since they are also inert, they have their own ode.
%> 
%> 
%> @retval dx Current differential equations vector of the system
%======================================================================
function dx = ode(x,u_ref)
    dx = x; 
    
    dx(1) =  x(2);
    dx(2) =  x(3);
    dx(3) = (u_ref(1)-x(3))/0.1; % T1 = 0.1 sek
 %   dx(3) = u_ref(1);
end
