function mpc_test
% TODO: NEEDS UPDATE; SWITCH TO RTI 6 AND REMOVED BATTERY VOLTAGE IN MODEL
    
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
    dt_MPC = 1/20;
    
    state = [1.5, 3.4, 0, 0];
    
    parameters = [ 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 ]';
    
    Hp = 6;
    Hu = 3;
    mpcController = MpcController(parameters, Hp, Hu, dt_MPC);
    
    u_delayed = zeros(3,3);
    
    for t_now = (0:dt:27)+1e-6        
        
        t_ref = [test_trajectory.t] * 1e-9;
        
        %% Interpolate reference trajectory
        reference_trajectory_x = [];
        reference_trajectory_y = [];
        for i_inter = 1:Hp
            
            t_interp = i_inter*dt_MPC + t_now;
            I = find(t_interp < t_ref);
            I = I(1);
            interp = TrajectoryInterpolation(t_interp, test_trajectory(I-1), test_trajectory(I));
            
            reference_trajectory_x(i_inter) = interp.position_x;
            reference_trajectory_y(i_inter) = interp.position_y;
            
        end
        
        
        
        
        
        [u_command, trajectory_pred_x, trajectory_pred_y] = ...
            mpcController.update(state, reference_trajectory_x, reference_trajectory_y);
        
        u_command(1) = max(-1,min(1,u_command(1))); % motor
        u_command(2) = max(-1,min(1,u_command(2))); % steering
        u_command(3) = 8; % battery voltage
        
        
        
        % Simulate
        state = state + dt * vehicle_dynamics(state,u_delayed(1,:),parameters);
        state = state + 1e-3 * randn(size(state));
        
        u_delayed = [u_delayed(2:end,:);u_command];
        
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

