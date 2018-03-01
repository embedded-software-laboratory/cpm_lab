function vehicle_friction_deceleration

    clc
    messages = read_json_log('vehicle_friction_deceleration_log.txt');
    
    T = [];
    X = [];
    Y = [];
    Theta = [];
    
    for message = messages
        if strcmp('cpm_msgs/VehicleObservation', message.type)
            T(end+1) = message.message.stamp_nanoseconds;
            X(end+1) = message.message.pose.position.x;
            Y(end+1) = message.message.pose.position.y;
            qz = message.message.pose.orientation.z;
            qw = message.message.pose.orientation.w;
            Theta(end+1) = atan2(qz, qw) * 2;
        end
    end
    
    T = (T - T(1))/1e9;
    
%     scatter(X,Y)
%     scatter(T,X)    
%     scatter(T,Y)
%     scatter(T,Theta)
%     scatter(1:length(diff(T)),diff(T))
    
    range = 255:292;
    
    T = T(range);
    X = X(range);
    Y = Y(range);
    Theta = Theta(range);
    
    S = cumsum([0 sqrt(diff(X).^2 + diff(Y).^2)]);
    scatter(T,S)
    
    N = 3;
    p = polyfit(T,S,N);
    
    t = sym('t');
    s = t.^(N:-1:0) * p';
    v = diff(s,t);
    a = diff(v,t);
    
    V = double(subs(v,linspace(min(T),max(T))));
    A = double(subs(a,linspace(min(T),max(T))));
    clf
    plot(V,A)
    hold on
    % manually fitted acceleration model a(v)
    plot(V,-1.3*tanh(30*V)-0.6*V)

end

