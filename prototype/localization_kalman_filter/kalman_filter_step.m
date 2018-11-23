function [x,P] = kalman_filter_step(x_predicted,z_predicted,z_measured,P,A,C,Q,R)    
    P = A * P * A' + Q;
    dz = z_measured - z_predicted;
    K = (P * C') / (C * P * C' + R);
    x = x_predicted + K * dz;
    P = (eye(size(A)) - K * C) * P;
end

