function system_fn = discretize_linearize(dt)

    px = sym('px', 'real');
    py = sym('py', 'real');
    yaw_offset = sym('yaw_offset', 'real');

    v = sym('v', 'real'); 
    yaw_imu = sym('yaw_imu', 'real');
    
    x = [px; py; yaw_offset];
    u = [v; yaw_imu];
    

    [dx, z] = model(x, u);
    
    A_c = jacobian(dx, x);
    B_c = jacobian(dx, u);
    C = double(jacobian(z, x));
    
    A_c_fn = matlabFunction(A_c,'Vars',{x,u});
    B_c_fn = matlabFunction(B_c,'Vars',{x,u});
    z_fn = matlabFunction(z,'Vars',{x,u});
    
    
    
    system_fn = @(X,U)output_helper(A_c_fn, z_fn, X, U, B_c_fn, C, dt);
end

function [x_next, Ad, Bd, C, z] = output_helper(A_c_fn, z_fn, x, u, B_c_fn, C, dt)
    
    Ac = A_c_fn(x,u);
    z = z_fn(x,u);
    Bc = B_c_fn(x,u);
    
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



