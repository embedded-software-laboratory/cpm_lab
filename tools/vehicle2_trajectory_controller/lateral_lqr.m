function lateral_lqr

    
    v = 1.0;
    T_steer = 0.5;
    
    A = [0 v 0; 0 0 v; 0 0 -1/T_steer];
    B = [0;0;1/T_steer];
    
    Q = eye(3);
    R = 1;
    N = zeros(size(B));
    
    [K,S,E] = lqr(A,B,Q,R,N)
    
    K(3) = 0;
    
    eig(A - B*K)
        
end

