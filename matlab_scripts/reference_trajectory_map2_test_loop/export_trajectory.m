function export_trajectory
     
    reference_path = dlmread('reference_path.csv');
    path_x = reference_path(:,1);
    path_y = reference_path(:,2);
    
    opt_result = load('optimization_checkpoint.mat');
    
    dx = circshift(path_x, -1, 1) - circshift(path_x, 1, 1);
    dy = circshift(path_y, -1, 1) - circshift(path_y, 1, 1);
    path_yaw = atan2(dy, dx);
    
    slice_time = 1:25:opt_result.p.Hp;
    dt_new = 25 * opt_result.p.dt;
    
    s = full(opt_result.Y_0(opt_result.p.idx_s));
    v = full(opt_result.Y_0(opt_result.p.idx_v));
    
    s = s(slice_time);
    v = v(slice_time);
    s_idx = round(s / opt_result.p.ds + 1);
    x = path_x(s_idx);
    y = path_y(s_idx);
    yaw = path_yaw(s_idx);
    vx = v .* cos(yaw);
    vy = v .* sin(yaw);
    t = ((1:length(slice_time))-1) * dt_new;
    T_loop = opt_result.p.Hp * opt_result.p.dt;
    T_gap = opt_result.p.delta_H_veh * opt_result.p.dt;
    
    clf
    hold on
    axis equal
    
    scatter(x,y)
    
    arrow_X = [x (x+0.1*vx) nan*x]';
    arrow_Y = [y (y+0.1*vy) nan*y]';
    
    plot(arrow_X(:), arrow_Y(:))
    
    cpp_file = fopen('test_loop_trajectory.hpp','w');
    
    fprintf(cpp_file, '#pragma once\n');
    fprintf(cpp_file, '#include "VehicleCommandTrajectory.hpp"\n');
    fprintf(cpp_file, 'static inline void get_test_loop_trajectory(\n');
    fprintf(cpp_file, '    std::vector<TrajectoryPoint> &points,\n');
    fprintf(cpp_file, '    uint64_t &loop_period_nanoseconds,\n');
    fprintf(cpp_file, '    uint64_t &point_period_nanoseconds,\n');
    fprintf(cpp_file, '    uint64_t &vehicle_time_gap_nanoseconds,\n');
    fprintf(cpp_file, '    int &n_max_vehicles\n');
    fprintf(cpp_file, ')\n');
    fprintf(cpp_file, '{\n');
    fprintf(cpp_file, '    loop_period_nanoseconds = %uull;\n', uint64(T_loop*1e9));
    fprintf(cpp_file, '    point_period_nanoseconds = %uull;\n', uint64(dt_new*1e9));
    fprintf(cpp_file, '    vehicle_time_gap_nanoseconds = %uull;\n', uint64(T_gap*1e9));
    fprintf(cpp_file, '    n_max_vehicles = %i;\n', opt_result.p.nVeh);
    fprintf(cpp_file, '    points = \n');
    fprintf(cpp_file, '    {\n');
    for i = 1:length(x)
        fprintf(cpp_file, '        TrajectoryPoint(TimeStamp(%uull), %f, %f, %f, %f),\n',...
            uint64(t(i)*1e9), x(i), y(i), vx(i), vy(i));
    end
    fprintf(cpp_file, '    };\n');
    fprintf(cpp_file, '}\n');

    fclose(cpp_file);
    
end

