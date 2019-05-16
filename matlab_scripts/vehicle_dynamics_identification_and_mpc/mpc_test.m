function mpc_test

    
    test_trajectory_tmp = read_rti_csv('test_trajectory.csv');
    for i=1:length(test_trajectory_tmp.px)
        test_trajectory(i).t = test_trajectory_tmp.t(i);
        test_trajectory(i).px = test_trajectory_tmp.px(i);
        test_trajectory(i).py = test_trajectory_tmp.py(i);
        test_trajectory(i).vx = test_trajectory_tmp.vx(i);
        test_trajectory(i).vy = test_trajectory_tmp.vy(i);
    end
    
    
    %interp = TrajectoryInterpolation(0.300, test_trajectory(2), test_trajectory(3))
    
    clf
    hold on
    
    scatter(test_trajectory_tmp.px, test_trajectory_tmp.py);    
    vehicle_patch = patch(0,0,0,'FaceColor','blue','FaceAlpha',.3);
    axis equal
    xlim([-1 5])
    ylim([-1 5])
    
    
    dt = 1/50;
    
    state = [2, 3.8, 0, 1, 0];
    
    parameters = [ 1.007419, -0.191607, 0.199668, 3.590788, -1.816570, -9.134298, 2.269441, 1.365857, 12.233076, 0.033411, -0.012818 ]';
    
    mpcController = MpcController(parameters);
    
    for t_now = (0:dt:3)+1e-6
        
        u = mpcController.update(state);
        
        state = state + dt * vehicle_dynamics(state,u,parameters);
        
        
        
        vehicle_geometry_x = [-1 1 1 -1]*0.1;
        vehicle_geometry_y = [1 1 -1 -1]*0.05;
        c = cos(state(3));
        s = sin(state(3));
        
        set(vehicle_patch,'XData',vehicle_geometry_x*c - vehicle_geometry_y*s + state(1));
        set(vehicle_patch,'YData',vehicle_geometry_x*s + vehicle_geometry_y*c + state(2));
        pause(1e-7)
        drawnow
    end
end

