function mpc_test(is_slow)

    if (is_slow)
        n_pts = 1000;
        dt = 0.25e9;
        v = 0.3;
        r = 1;
        c_x = 2;
        c_y = 2.25;
        t_full = 2e9*pi*r/v;
        t = 0:dt:n_pts*dt;
        test_trajectory = struct('t',num2cell(t)...
            ,'px',num2cell(c_x + r*cos(t/t_full * 2*pi))...
            ,'py',num2cell(c_y + r*sin(t/t_full * 2*pi))...
            ,'vx',num2cell(-sin(t/t_full * 2*pi))...
            ,'vy',num2cell( cos(t/t_full * 2*pi))...
        );
        
        state = [c_x+r, c_y, pi/2, 0];
    else
        test_trajectory_tmp = read_rti_csv('test_trajectory.csv');
        for i=1:length(test_trajectory_tmp.px)
            test_trajectory(i).t = test_trajectory_tmp.t(i);
            test_trajectory(i).px = test_trajectory_tmp.px(i);
            test_trajectory(i).py = test_trajectory_tmp.py(i);
            test_trajectory(i).vx = test_trajectory_tmp.vx(i);
            test_trajectory(i).vy = test_trajectory_tmp.vy(i);
        end
        
        state = [2.26, 3.75, 0, 0];
    end
    
    
    
    
    clf
    hold on
    
    scatter([test_trajectory.px], [test_trajectory.py],1);
    vehicle_patch = patch(0,0,0,'FaceColor','blue','FaceAlpha',.3);
    plot_ref_trajectory = plot(0,0,'b','LineWidth',3);
    plot_pred_trajectory = plot(0,0,'r','LineWidth',3);
    axis equal
    xlim([0 4.5])
    ylim([0 4])
    
    
    dt = 1/50;
    dt_MPC = 1/20;
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

