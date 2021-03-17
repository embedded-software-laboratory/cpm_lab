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


function mpc = generate_mpc_matrices( scenario,iter )

    % Compute all relevant discretization and MPC matrices for all
    % vehicles.
        
    nx = scenario.model.nx;
    nu = scenario.model.nu;
    ny = scenario.model.ny;
    Hp = scenario.Hp;
    Hu = scenario.Hu;
    dt = scenario.dt;

    mpc=struct;
    
    %% GENARATE MATRICES
    for i=1:Hp
       mpc.Reference( blk(i,ny),1 ) = iter.RefData.ReferenceTrajectoryPoints(:,i);
    end
    
    mpc.A = zeros(nx,nx,Hp);
    mpc.B = zeros(nx,nu,Hp);
    mpc.E = zeros(nx,Hp);
    
    [ A,B,C,E ] ...
        = discretize( iter.x0(:), iter.u0(:), dt, scenario.model );
    [ Psi, Gamma, mpc.Theta, Pie ] ...
        = prediction_matrices( A,B,C,nx,nu,ny,Hp,Hu );
    
    mpc.freeResponse = Psi *iter.x0(:) + Gamma *iter.u0(:) + Pie*E;
    
    [ mpc.H , mpc.g, mpc.r ] ...
        = mpc_cost_function_matrices( scenario.Q, scenario.R, nu,ny,Hp,Hu,mpc,scenario.Q_final );
    
    % For the simple linearization, the same A,B,E are used in
    % every prediction step.
    for i=1:Hp
        mpc.A(:,:,i) = A;
        mpc.B(:,:,i) = B;
        mpc.E(:,i) = E;
    end
