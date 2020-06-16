function animate_trajectory
        
    reference_path = dlmread('reference_path.csv');
    path_x = reference_path(:,1);
    path_y = reference_path(:,2);
    
    opt_result = load('optimization_checkpoint.mat');
    
    clf
    hold on
    plot(path_x, path_y);
    axis equal
    h = plot(0,0);
    plot(4.5*[0 0 1 1 0],4*[0 1 1 0 0],'k','LineWidth',4)

    position_fn = full(opt_result.s);
    
    k = 1;
    while true
        vehicle_x = [];
        vehicle_y = [];
        
        circle_c = [0.1*cos(linspace(0,2*pi,100)) nan];
        circle_s = [0.1*sin(linspace(0,2*pi,100)) nan];
        
        for i = 1:opt_result.p.nVeh
            
            
            t_idx = opt_result.p.delta_H_veh * i + k;
            s = circshift(position_fn, t_idx, 1);
            s_idx = s(1) / opt_result.p.ds + 1;
            s_idx = round(s_idx);
            
            tmp_x = circshift(path_x, s_idx, 1);
            tmp_y = circshift(path_y, s_idx, 1);
            
            vehicle_x = [vehicle_x (circle_c+tmp_x(1))];
            vehicle_y = [vehicle_y (circle_s+tmp_y(1))];
        end
        
        set(h, 'XData', vehicle_x);
        set(h, 'YData', vehicle_y);
        drawnow
        k = k + 2;
    end
    
end

