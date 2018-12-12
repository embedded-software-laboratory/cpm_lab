function make_test_trajectory

    path = load('path');
    
    x = path.x / 1000;
    y = path.y / 1000;
    yaw = path.phi;
    curvature = path.path(:,4) * 1000;
    
    slice = 8925:17576;
    x = x(slice);
    y = y(slice);
    yaw = yaw(slice);
    curvature = curvature(slice);
    curvature(1:100) = curvature(100);
    
    s = [0; cumsum(sqrt(diff(x).^2+diff(y).^2))];
    s2 = linspace(min(s), max(s), length(s));
    
    x = interp1(s,x,s2);
    y = interp1(s,y,s2);
    yaw = interp1(s,yaw,s2);
    curvature = interp1(s,curvature,s2);
    s = s2;
    
    
    %% find maximum speed
    a_max = 2;
    speed_max = 1.5;
    speed = min(speed_max, sqrt(a_max ./ (abs(curvature)+1e-5)));
    
    N = length(x);
    ds = s(2) - s(1);
    
    for i = 1:N
        i2 = mod(i, N)+1;
        delta_v = ds/speed(i) * sqrt(max(0,a_max^2 - speed(i)^4 * curvature(i)^2));
        speed(i2) = min(speed(i2), speed(i) + delta_v);
    end
    for i = fliplr(1:N)
        i2 = mod(i-2,N)+1;
        delta_v = ds/speed(i) * sqrt(max(0,a_max^2 - speed(i)^4 * curvature(i)^2));
        speed(i2) = min(speed(i2), speed(i) + delta_v);
    end
    
    
%     clf 
%     hold on
%     plot(s,speed)
%     
%     dv = circshift(speed,-1) - speed;
%     a = sqrt(speed.^4 .* curvature.^2 + (dv./ds .* speed).^2);
%     plot(s,a)
%     plot(s,curvature)
    
    %% convert to trajectory
    vx = speed .* cos(yaw);
    vy = speed .* sin(yaw);
    t = cumsum(ds ./ speed);
    t_total = sum(ds ./ speed);
    
    slice = 20:700:N;
    M = length(slice);
    
    s = s(slice);
    x = x(slice);
    y = y(slice);
    vx = vx(slice);
    vy = vy(slice);
    t = t(slice);
    yaw = yaw(slice);
    %plot(reshape([x;x+.1*vx;nan*x],3*M,1),reshape([y;y+.1*vy;nan*x],3*M,1))
    
    t = t - t(1);
    
    f = fopen('trajectory.h', 'w');
    fprintf(f,'const double example_trajectory_timestamp_offset[] = { %s };\n',sprintf('%d, ',t));
    fprintf(f,'const double example_trajectory_px[] = { %s };\n',sprintf('%d, ',x));
    fprintf(f,'const double example_trajectory_py[] = { %s };\n',sprintf('%d, ',y));
    fprintf(f,'const double example_trajectory_vx[] = { %s };\n',sprintf('%d, ',vx));
    fprintf(f,'const double example_trajectory_vy[] = { %s };\n',sprintf('%d, ',vy));
    fprintf(f,'const double example_trajectory_period = %d;\n',t_total);
    fprintf(f,'const int example_trajectory_size = %i;\n', M);
    fclose(f);

end

