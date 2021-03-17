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

% function to transform du into process variables Y
function [Traj,U] = decode_deltaU(scenario,iter,mpc,du_result)

Hp = scenario.Hp;
Hu = scenario.Hu;
nu = scenario.model.nu;
ny = scenario.model.ny;
U = zeros(nu,Hp);
du = reshape(du_result,[nu,Hu]);

% Control values

U(:,1) = du(:,1) + iter.u0;
for k=2:Hu
    U(:,k) = du(:,k) + U(:,k-1);
end
for k=Hu+1:Hp
    U(:,k) = U(:,Hu);
end


% Predicted trajectory

Y = mpc.freeResponse + mpc.Theta*du_result;
Y=reshape(Y,[scenario.model.ny,Hp]);
Traj = Y(1:2,:); % 1:2 corresponds to pos and vel value


