function system_fn = discretize_linearize(dt)

    px = sym('px', 'real');
    py = sym('py', 'real');
    yaw = sym('yaw', 'real');
    yaw_offset = sym('yaw_offset', 'real');
    s = sym('s', 'real');
    v = sym('v', 'real');    
    curvature = sym('curvature', 'real');    
    
    v_ref = sym('v_ref', 'real');    
    curvature_ref = sym('curvature_ref', 'real');    
    
    x = [px; py; yaw; yaw_offset; s; v; curvature];
    u = [v_ref; curvature_ref];
    

    [dx, z_L, z_P] = model(x, u);
    
    A_c = jacobian(dx, x);
    B_c = double(jacobian(dx, u));
    C_L = double(jacobian(z_L, x));
    C_P = double(jacobian(z_P, x));
    
    A_c_fn = matlabFunction(A_c,'Vars',{x});
    z_L_fn = matlabFunction(z_L,'Vars',{x});
    z_P_fn = matlabFunction(z_P,'Vars',{x});
    
    
    
    system_fn = @(X,U)output_helper(A_c_fn, z_L_fn, z_P_fn, X, U, B_c, C_L, C_P, dt);
end

function [x_next, Ad, Bd, CL, CP, zL, zP] = output_helper(A_c_fn, z_L_fn, z_P_fn, x, u, Bc, CL, CP, dt)
    
    Ac = A_c_fn(x);
    zL = z_L_fn(x);
    zP = z_P_fn(x);    
    
    [Ad, Bd] = c2d_expm(Ac, Bc, dt);
    x_next = Ad*x + Bd*u;
    
end

function [Ad, Bd] = c2d_expm(Ac, Bc, dt)
    M = [Ac Bc];
    M(length(M),length(M)) = 0;
    M = expm(M*dt);
    Ad = M(1:size(Ac,1),1:size(Ac,2));
    Bd = M(1:size(Bc,1),size(Ac,2)+1:end);
end



