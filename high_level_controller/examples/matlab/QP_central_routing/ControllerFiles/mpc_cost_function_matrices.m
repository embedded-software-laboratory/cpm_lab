% Copyright 2016 Bassam Alrifaee
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as 
% published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

function [ H, g, r ] = mpc_cost_function_matrices( Q_weight, R_weight, nu,ny,Hp,Hu,mpc,Q_final )

    % Compute G and H matrices in the quadratic cost function.
    Q = diag(repmat(Q_weight,1,Hp));
    Q(end-ny+1:end, end-ny+1:end) = diag(Q_final);
    
    R = diag(repmat(R_weight,1,Hu));
    Error = mpc.Reference - mpc.freeResponse;    
    % make symmetric if Q or R werent symmetric
    H = symmetric(mpc.Theta'*Q*mpc.Theta + R); 
    g = -2*mpc.Theta'*Q*Error;

    r = Error' * Q * Error;
end

function A = symmetric( A )
A=.5*(A+A');
end

