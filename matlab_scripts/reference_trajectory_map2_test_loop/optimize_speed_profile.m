function optimize_speed_profile
    
    collision_map = load_collision_map();
    p = struct;
    p.dt = 0.01;
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
    
    
    Y_init = nan(p.Hp, 1);
    Y_init(p.idx_s) = linspace(p.s_min, p.s_max, p.Hp);
    Y_init(p.idx_v) = p.v_ref;
    Y_init(p.idx_a) = 0;
    
    D = -diag(ones(p.Hp,1)) + diag(ones(p.Hp-1,1),1);
    D = sparse(D(1:end-1,:));
    D = D / p.dt;
    
    Aeq = [];
    beq = [];
    
    Aeq = [Aeq; [D -speye(p.Hp-1,p.Hp) sparse(p.Hp-1,p.Hp)]];
    beq = [beq; sparse(p.Hp-1,1)];
    
    Aeq = [Aeq; [sparse(p.Hp-1,p.Hp) D -speye(p.Hp-1,p.Hp) ]];
    beq = [beq; sparse(p.Hp-1,1)];
    
    Aeq(end+1,p.idx_s(1)) = 1;
    beq(end+1,1) = p.s_min;
    
    Aeq(end+1,p.idx_s(end)) = 1;
    beq(end+1,1) = p.s_max;
    
    Aeq(end+1,p.idx_v(1)) = 1;
    beq(end+1,1) = p.v_ref;
    
    Aeq(end+1,p.idx_v(end)) = 1;
    beq(end+1,1) = p.v_ref;
    
    KKT_mat = [speye(size(Aeq,2), size(Aeq,2)) Aeq'; Aeq speye(size(Aeq,1), size(Aeq,1))];
    
    
    
    Y_new_A = Y_init + 1e-5 * randn(size(Y_init));
    
    kkt_soln = KKT_mat \ [Y_new_A;beq];
    Y_new_B = kkt_soln(1:size(Aeq,2));
    
    kkt_soln = KKT_mat \ [Y_new_B;beq];
    Y_new_B = kkt_soln(1:size(Aeq,2));
    
    plot(Aeq*Y_init-beq,'o-')
    plot(Aeq*Y_new_A-beq,'o-')
    plot(Aeq*Y_new_B-beq,'o-')
    
    
    %o=objective_fn(Y_init, collision_map, p);
    
    %Y_soln = fmincon(@(Y) objective_fn(Y, collision_map, p) , Y_init, [], []);
    
    format long
    format compact 
    
%     lb = [];
%     ub = [];
%     options = optimoptions('fmincon', ...
%         'Display','iter-detailed', ...
%         'DiffMinChange',2*p.ds, ...
%         'FiniteDifferenceStepSize',2*p.ds ...
%     );
%     
%     Y_soln = fmincon(@(Y) objective_fn(Y, collision_map, p) , Y_init, [], [], Aeq, beq, lb, ub, [], options);
    
end

function objective = objective_fn(Y, collision_map, p)

    assert(size(Y,2) == 1);

    s = Y(p.idx_s);
    v = Y(p.idx_v);
    a = Y(p.idx_a);
    
    objective = mean(a.^2);
    objective = objective + mean((v - p.v_ref).^2);
    
    s1q = [];
    s2q = [];    
    for i = 1:(p.nVeh-1)
        s2 = circshift(s, p.delta_H_veh * i, 1);
        s1q = [s1q; s];
        s2q = [s2q; s2];
    end
    
    collision_penalties = interp2(collision_map, s1q / p.ds + 1, s2q / p.ds + 1);
    objective = objective + 100 * mean(collision_penalties);
end



