function mpc_test

    
    test_trajectory_tmp = read_rti_csv('test_trajectory.csv');
    for i=1:length(test_trajectory_tmp.px)
        test_trajectory(i).t = test_trajectory_tmp.t(i);
        test_trajectory(i).px = test_trajectory_tmp.px(i);
        test_trajectory(i).py = test_trajectory_tmp.py(i);
        test_trajectory(i).vx = test_trajectory_tmp.vx(i);
        test_trajectory(i).vy = test_trajectory_tmp.vy(i);
    end
    
    
    
    clf
    hold on
    
    scatter(test_trajectory_tmp.px, test_trajectory_tmp.py);    
    vehicle_patch = patch(0,0,0,'FaceColor','blue','FaceAlpha',.3);
    plot_ref_trajectory = plot(0,0,'b');
    plot_pred_trajectory = plot(0,0,'r');
    axis equal
    xlim([-1 5])
    ylim([-1 5])
    
    
    dt = 1/50;
    
    state = [2, 3.8, 0, 1, 0];
    
    parameters = [ 1.007419, -0.191607, 0.199668, 3.590788, -1.816570, -9.134298, 2.269441, 1.365857, 12.233076, 0.033411, -0.012818 ]';
    
    Hp = 50;
    Hu = 25;
    mpcController = MpcController(parameters, Hp, Hu);
    
    for t_now = (0:dt:10)+1e-6
        
        
        t_ref = [test_trajectory.t]* 1e-9;
        
        %% Interpolate reference trajectory
        reference_trajectory_x = [];
        reference_trajectory_y = [];
        for i_inter = 1:Hp
            
            t_interp = i_inter*dt + t_now;
            I = find(t_interp < t_ref);
            I = I(1);
            interp = TrajectoryInterpolation(t_interp, test_trajectory(I-1), test_trajectory(I));
            
            reference_trajectory_x(i_inter) = interp.position_x;
            reference_trajectory_y(i_inter) = interp.position_y;
            
        end
        
        
        
        
        
        [u, trajectory_pred_x, trajectory_pred_y] = ...
            mpcController.update(state, reference_trajectory_x, reference_trajectory_y);
        
        u(1) = max(-1,min(1,u(1))); % motor
        u(2) = max(-1,min(1,u(2))); % steering
        u(3) = 8; % battery voltage
        
        
        % Simulate
        state = state + dt * vehicle_dynamics(state,u,parameters);
        
        
        % Visualize
        vehicle_geometry_x = [-1 1 1 -1]*0.1;
        vehicle_geometry_y = [1 1 -1 -1]*0.05;
        c = cos(state(3));
        s = sin(state(3));
        
        set(vehicle_patch,'XData',vehicle_geometry_x*c - vehicle_geometry_y*s + state(1));
        set(vehicle_patch,'YData',vehicle_geometry_x*s + vehicle_geometry_y*c + state(2));
        
        set(plot_ref_trajectory, 'XData', reference_trajectory_x);
        set(plot_ref_trajectory, 'YData', reference_trajectory_y);  
        
        set(plot_pred_trajectory, 'XData', trajectory_pred_x);
        set(plot_pred_trajectory, 'YData', trajectory_pred_y);        
        
        pause(1e-7)
        drawnow
    end
end

