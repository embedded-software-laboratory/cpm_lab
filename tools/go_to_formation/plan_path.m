function [path_pts, is_valid] = plan_path(vehiclePoses, iEgoVeh, goalPose)
    % vehiclePoses: 3-by-nVeh double
    % iEgoVeh: 1-by-1 uint, index of vehicle to plan trajectory
    % goalPose: 3-by-1 double
    % path_pts: 3-by-n_path_pts matrix [m, m, deg]
    % is_valid: bool

    costmap = setCostmap(vehiclePoses, iEgoVeh);
    %% Path Planning
    startPose = vehiclePoses(:,iEgoVeh);
    helpPose = goalPose - 0.6*[cosd(goalPose(3)); sind(goalPose(3)); 0];

    planner = pathPlannerRRT(costmap,...
        'GoalTolerance', [0.01 0.01 3],...
        'MinTurningRadius', 0.5,...
        'ConnectionMethod', 'Dubins', ...
        'MaxIterations', 2000 ...
    );
    
    try
        path_rrt_1 = plan(planner,startPose',helpPose');
        path_rrt_2 = plan(planner,helpPose',goalPose');
        path_rrt = [path_rrt_1, path_rrt_2];
        path_pts = rrt_path_to_poses(path_rrt);
        is_valid = true;
    catch
        is_valid = false;
        path_pts = [];
    end
end

function path_pts = rrt_path_to_poses(path_rrt)
    ds = 0.005;
    path_pts = [];
    for irp = 1:numel(path_rrt)
        rp = path_rrt(irp);
        curLengths = 0:ds:rp.Length; % Path length between two intermitting poses.
        addAllPoses = interpolate(rp,curLengths);
        if irp>1
            % remove double poses on concatenation
            addAllPoses = addAllPoses(2:end,:);
        end
        path_pts = [path_pts addAllPoses']; %#ok<AGROW>
    end
end