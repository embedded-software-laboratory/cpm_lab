function [obstacle_trajectory, time_cpm]= obstacle2trajectory(Obstacle_Data, dt_commonroad, dt_cpm)

    % interpolates obstacle from commonroad time steps to cpm time steps
    % only position. ignores velocity, orientation etc.

    numObstacle = Obstacle_Data.numObst;
    max_numTimesteps_cr= 0;
    
    % check if we have at least on dynamics obstacle.
    for   obst = 1: numObstacle
        if strcmp(Obstacle_Data.Obstacle{1,obst}.role,'dynamic')
            max_numTimesteps_cr = max(max_numTimesteps_cr,  size(Obstacle_Data.Obstacle{1,obst}.trajectory,1));
            break;
        end
    end
    
    if 0==max_numTimesteps_cr
       % only static obstacles
       max_numTimesteps_cr = 2;
    end
    
    % sampling point of cpm software
    t_end =  dt_commonroad*(max_numTimesteps_cr-1);
    time_cr = 0 : dt_commonroad : t_end;  
    numTimesteps_cpm = ceil(t_end/dt_cpm)+1;
    time_cpm = 0:dt_cpm : (numTimesteps_cpm-1)*dt_cpm;
    
    % x and y values of trajectory
    trajectory_cr = NaN(max_numTimesteps_cr,2,numObstacle);
    trajectory_cpm = NaN(numTimesteps_cpm,2,numObstacle);
    obstacle_trajectory =cell(1,numObstacle);

        
    for obst = 1: numObstacle
        
        switch Obstacle_Data.Obstacle{1,obst}.role
        
            case 'static'
                
                
            case 'dynamic'
                thisObst_numTimesteps = size(Obstacle_Data.Obstacle{1,obst}.trajectory,1);
                for ts = 1:thisObst_numTimesteps
                    trajectory_cr (ts,:,obst) = Obstacle_Data.Obstacle{1,obst}.trajectory{ts, 1}.position  ;
                end
                % zero order hold for extrapolation = stopped obstacle
                for ts = thisObst_numTimesteps+1 : max_numTimesteps_cr
                    trajectory_cr (ts,:,obst) = trajectory_cr (thisObst_numTimesteps,:,obst);
                end
                
            otherwise
                error(['unknown case: ', Obstacle_Data.Obstacle{1,obst}.role])
        end
        % interpolate between timesteps 
        trajectory_cpm(:,1,obst) = interp1(time_cr,trajectory_cr(:,1,obst),time_cpm,'linear','extrap');
        trajectory_cpm(:,2,obst) = interp1(time_cr,trajectory_cr(:,2,obst),time_cpm,'linear','extrap');
        obstacle_trajectory{1,obst} = trajectory_cpm(:,:,obst)';
        
        
    end
    
    
end