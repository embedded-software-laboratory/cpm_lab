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

function optimize_speed_profile
    
    [collision_map, collision_dist_sq] = load_collision_map();
    p = struct;
    p.dt = 0.01;
    p.ds = 0.01;
    p.Hp = 3000;
    p.nVeh = 15;
    p.delta_H_veh = 200;
    p.s_min = 0;
    p.s_max = 36.49;
    p.v_min = 0.2;
    p.v_max = 2.0;
    p.a_min = -0.3;
    p.a_max = 0.3;
    p.v_ref = p.s_max/((p.Hp-1)*p.dt);
    p.idx_s = (1:p.Hp);
    p.idx_v = (1:p.Hp) + p.Hp;
    p.idx_a = (1:p.Hp) + 2 * p.Hp;
    
    %% Initial guess
    if exist('optimization_checkpoint.mat','file')
        tmp = load('optimization_checkpoint.mat');
        Y_init = tmp.Y_0;
    else
        Y_init = nan(p.Hp, 1);
        Y_init(p.idx_s) = linspace(p.s_min, p.s_max, p.Hp);
        Y_init(p.idx_v) = p.v_ref;
        Y_init(p.idx_a) = 0;
    end
    
    %% Finite differences
    D = -diag(ones(p.Hp,1)) + diag(ones(p.Hp-1,1),1);
    D = sparse(D(1:end-1,:));
    D = D / p.dt;
    
    %% Equation system
    Aeq = [];
    beq = [];
    
    % dot x == v
    Aeq = [Aeq; [D -speye(p.Hp-1,p.Hp) sparse(p.Hp-1,p.Hp)]];
    beq = [beq; sparse(p.Hp-1,1)];
    
    % dot v == a
    Aeq = [Aeq; [sparse(p.Hp-1,p.Hp) D -speye(p.Hp-1,p.Hp) ]];
    beq = [beq; sparse(p.Hp-1,1)];
    
    % s(0) == s_min
    Aeq(end+1,p.idx_s(1)) = 1;
    beq(end+1,1) = p.s_min;
    
    % s(end) == s_max
    Aeq(end+1,p.idx_s(end)) = 1;
    beq(end+1,1) = p.s_max;
    
    % v(0) == v_ref
    Aeq(end+1,p.idx_v(1)) = 1;
    beq(end+1,1) = p.v_ref;
    
    % v(end) == v_ref
    Aeq(end+1,p.idx_v(end)) = 1;
    beq(end+1,1) = p.v_ref;
    
    
    %% KKT system for reprojecting to the equation system
    kkt_mat = [speye(size(Aeq,2), size(Aeq,2)) Aeq'; Aeq sparse(size(Aeq,1), size(Aeq,1))];
    
    
    Y_0 = Y_init;
    objective_0 = objective_fn(Y_0, collision_map, collision_dist_sq, p);
    
    while true
        for i = 1:100
            Y_new = Y_0 + 1e-4 * randn(size(Y_0));
            
            % smooth acceleration
            accel = Y_new(p.idx_a);
            accel(2:end-1) = (accel(1:end-2) + accel(2:end-1) + accel(3:end)) / 3;
            Y_new(p.idx_a) = accel;
            
            
            kkt_soln = kkt_mat \ [Y_new;beq];
            Y_new = kkt_soln(1:size(Aeq,2));
            [objective_new, min_dist] = objective_fn(Y_new, collision_map, collision_dist_sq, p);

            if objective_new < objective_0
                objective_0 = objective_new;
                Y_0 = Y_new;
                fprintf('%f, %f\n', objective_0, min_dist);
            end
        end

        plot(Y_0);

        s = Y_0(p.idx_s);
        save('optimization_checkpoint','Y_0','s','p');
        drawnow
        pause(0.01);
    end
    
end

function [objective, min_dist] = objective_fn(Y, collision_map, collision_dist_sq, p)

    assert(size(Y,2) == 1);

    s = Y(p.idx_s);
    v = Y(p.idx_v);
    a = Y(p.idx_a);
    
    objective = 10*mean(a.^2);
    objective = objective + mean((v - p.v_ref).^2);
    
    s1q = [];
    s2q = [];    
    for i = 1:(p.nVeh-1)
        s2 = circshift(s, p.delta_H_veh * i, 1);
        s1q = [s1q; s];
        s2q = [s2q; s2];
    end
    
    collision_penalties = interp2(collision_map, s1q / p.ds + 1, s2q / p.ds + 1);
    collision_dists_sq_interp = interp2(collision_dist_sq, s1q / p.ds + 1, s2q / p.ds + 1);
    min_dist = sqrt(min(collision_dists_sq_interp));
    objective = objective + 100 * mean(collision_penalties);
end



