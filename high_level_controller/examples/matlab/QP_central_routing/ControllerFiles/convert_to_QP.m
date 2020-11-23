
% function to generate constraint matrices for the QP Problem. 
function qp_problem = convert_to_QP(scenario,iter,mpc)

%% RESULT FORM:
%   x = [ du1; du2;...;duHu];
%   minimize    x'*p0*x + q0'*x
%               Ax -b <= 0
%% PARAMETERS
nu = scenario.model.nu;
ny = scenario.model.ny;
Hp = scenario.Hp;
Hu = scenario.Hu;
u_prev = iter.u0;
idx_pos=1:2;
H_coll = scenario.H_coll;

% num of Vehicles with higher priority
nObst = scenario.nObst;

%% project collision points into local coordinates
full_traj_pos = iter.RefData.TrajPointsGlobal; % global coordinates of sampling points. first point is currPos = pos(k=0)
diff_full_traj_pos  = full_traj_pos(:,2:end)-full_traj_pos(:,1:end-1);
diff_s = [0,sqrt(diff_full_traj_pos(1,:).^2 + diff_full_traj_pos(2,:).^2)];% distance between sampling po

local_s =  [cumsum(diff_s)] ; % distance to current position

n_trajectoryPoints = min(size(full_traj_pos,2),3*Hp);
s_goal_max = local_s(end); % constraint when we reach goal


if nObst >0
    
    %% safety distance static, given by vehicle diameter
    dsafeExtra = 0.001;
    R_safe= (dsafeExtra + scenario.dsafeVehicles);
    s_obst_max = local_s(end)*ones(H_coll,nObst); % maximum allowed progress in each time step as upper bound.
    
    % collisionPoints{k} = trajectory points of vehicle k in global coordinate
    % first point is predicted position in this time step
    p_clb1 = zeros(2,H_coll,nObst);
    p_clb2 = zeros(2,H_coll,nObst);
    p_crb1 = zeros(2,H_coll,nObst);
    p_crb2 = zeros(2,H_coll,nObst);
    p_circ  = zeros(2,H_coll+1,nObst);
    p_t = full_traj_pos; % reference trajectory in global coordinates
    for obst =1:nObst
         pos_c = scenario.dynamicObstacles{obst}; % first point corresponds to k=0
         if isempty(pos_c)
            continue; % catches the case where some vehicles havent started yet
         end

         % direction vector
         diff_points = pos_c(:,2:end) - pos_c(:,1:end-1);
         diff_points =diff_points ./ sqrt(diff_points(1,:).^2 + diff_points(2,:).^2+1e-16);
         
         % orthogonal direction is direction for shift
         shift_direction = [-diff_points(2,:); diff_points(1,:)]; % 
         
         % total safety diameter
        p_clb1(:,:,obst) = pos_c(idx_pos,1:H_coll) + R_safe* shift_direction(:,1:H_coll); % collision left bound
        p_crb1(:,:,obst) = pos_c(idx_pos,1:H_coll) - R_safe* shift_direction(:,1:H_coll); % collision right bound
        p_clb2(:,:,obst) = pos_c(idx_pos,2:H_coll+1) + R_safe* shift_direction(:,1:H_coll); % collision left bound
        p_crb2(:,:,obst) = pos_c(idx_pos,2:H_coll+1) - R_safe* shift_direction(:,1:H_coll); % collision right bound
        p_circ(:,:,obst) = pos_c(idx_pos,1:H_coll+1);% center of circle
        
        
        for k = 1:H_coll % time steps. m=1 is timestep k=0
            
           lambda_max = 42; % reset. > 1 means not active.
            
            
            p_c1 = p_circ(:,k,obst);
            p_c2 = p_circ(:,k+1,obst);
            
            p_r1 = p_crb1(:,k,obst);
            p_r2 = p_crb2(:,k,obst);
            p_l1 = p_clb1(:,k,obst);
            p_l2 = p_clb2(:,k,obst);
            
            for j = 1:n_trajectoryPoints-1
                p_t1 = p_t(:,j);
                p_t2 = p_t(:,j+1);
                
                
                if (p_t2-p_t1)'*(p_t2-p_t1) < 1e-6
                    % points too close. can happen if we reach end of
                    % trajectory and want to stop.
                    continue;
                end
                % solve linear system of equation for left bound
                if (p_l2-p_l1)'*(p_l2-p_l1) > 1e-6 % if obstacle is static in this point, we can skip this rectangle part
                    A_clb = [p_l1-p_l2, p_t2-p_t1];
                    b_clb = [p_l1-p_t1];
                   
                    
                    if rank(A_clb) ==2
                        lambda_clb = A_clb\b_clb; %[lambda_l; lambda_t]
                        lambda_l = lambda_clb(1);
                        lambda_tl = lambda_clb(2);
                        if 0<= lambda_tl && lambda_tl <= 1 && 0<= lambda_l && lambda_l <= 1
                            lambda_max = min(lambda_max,lambda_tl);
                        end
                    else % parallel lines, no collision here (being exactly on collision line is catched by d_safe_extra
                        
                    end
                end
                % solve linear system of equation for right bound
                if (p_r2-p_r1)'*(p_r2-p_r1) > 1e-6 % if obstacle is static in this point,, we can skip this rectangle part
                    A_crb = [p_r1-p_r2, p_t2-p_t1];
                    b_crb = [p_r1-p_t1];
                    if rank(A_clb) ==2
                        lambda_crb = A_crb\b_crb; %[lambda_r; lambda_t]
                        lambda_r = lambda_crb(1);
                        lambda_tr = lambda_crb(2);
                        if 0<= lambda_tr && lambda_tr <= 1 && 0<= lambda_r && lambda_r <= 1
                            lambda_max = min(lambda_max,lambda_tr);
                            
                        end
                    else % parallel lines, no collision here (being exactly on collision line is catched by d_safe_extra
                        
                    end
                end
                % solve quadratic equation for circle 1
                a2= (p_t2-p_t1)'*(p_t2-p_t1);
                a1 = 2*(p_t1-p_c1)'*(p_t2-p_t1);
                a0 = (p_t1-p_c1)'*(p_t1-p_c1) - R_safe^2;
                p = a1/a2;
                q= a0/a2;
                discr = p^2/4-q;
                if discr >= 0
                    lambda_1 = -p/2 + sqrt(discr);
                    lambda_2 = -p/2 - sqrt(discr);
                    % check if one lambda is in [0,1] -> collision possible
                    if (0<= lambda_1 && lambda_1 <= 1) || (0<= lambda_2 && lambda_2 <= 1)
                        if (0<= lambda_1 && lambda_1 <= 1) && (0<= lambda_2 && lambda_2 <= 1)
                            lambda_t = min(lambda_1,lambda_2); % first collision in [0,1]
                        else
                            if (0<= lambda_1 && lambda_1 <= 1)
                                lambda_t = lambda_1;
                            else
                                lambda_t = lambda_2;
                            end
                        end
                        lambda_max = min(lambda_max,lambda_t);
                    end
                    
                else
                    % no collision with this end
                end
                
                
                % solve quadratic equation for circle 2
                a2= (p_t2-p_t1)'*(p_t2-p_t1);
                a1 = 2*(p_t1-p_c2)'*(p_t2-p_t1);
                a0 = (p_t1-p_c2)'*(p_t1-p_c2) - R_safe^2;
                p = a1/a2;
                q= a0/a2;
                discr = p^2/4-q;
                if discr >= 0
                    lambda_1 = -p/2 + sqrt(discr);
                    lambda_2 = -p/2 - sqrt(discr);
                    % check if one lambda is in [0,1] -> collision possible
                    if (0<= lambda_1 && lambda_1 <= 1) || (0<= lambda_2 && lambda_2 <= 1)
                        if (0<= lambda_1 && lambda_1 <= 1) && (0<= lambda_2 && lambda_2 <= 1)
                            lambda_t = min(lambda_1,lambda_2); % first collision in [0,1]
                        else
                            if (0<= lambda_1 && lambda_1 <= 1)
                                lambda_t = lambda_1;
                            else
                                lambda_t = lambda_2;
                            end
                        end
                        lambda_max = min(lambda_max,lambda_t);
                    end
                    
                else
                    % no collision with this end
                end
                if 0<= lambda_max && lambda_max <=1
                    % convert lambda_max to s_max
                    s_obst_max(k,obst) = local_s(j) + lambda_max* diff_s(j);

                    break; % go to next iteration, this is s_max
                end
           end
       end
       
   end
   
    
end


%%  GENERATE MATRICES
% OBJECTIVE FUNCTION MATRICES


p0 = mpc.H;
q0 = mpc.g;
r0 = 0;


Theta_Pos = mpc.Theta(1:2:end,:); % y_pos prediction
freeResp_Pos = mpc.freeResponse(1:2:end); 
A_goal = Theta_Pos(1:H_coll,:);
b_goal = s_goal_max * ones(H_coll,1) - freeResp_Pos(1:H_coll);


A_coll = [];
b_coll = [];
if nObst> 0
    % Theta *delta_u < f - s_max
    A_coll = repmat(Theta_Pos(1:H_coll,:),[nObst,1]);
    b_coll = reshape(s_obst_max,[H_coll*nObst,1]) - repmat(freeResp_Pos(1:H_coll),[nObst,1]);
end 


% CONSTRAINTS MATRIX A
    % du_lim_min <= du <= du_lim_max
    ub_du_max = repmat([scenario.du_lim_max],Hu,1);
    lb_du_min = repmat([scenario.du_lim_min],Hu,1);
    
    % u_lim_min <= u <= u_lim_max
    pattern = repmat(eye(nu),[Hu,Hu]);
    A_u = [tril(pattern); -tril(pattern)];
    
    b_u_max = repmat([scenario.u_lim_max- u_prev] ,Hu,1);
    b_u_min = repmat([u_prev - scenario.u_lim_min] ,Hu,1);
  
    % constraint velocity >0
    Theta_Vel  = mpc.Theta(2:2:end,:); % vel prediction
    freeResp_Vel = mpc.freeResponse(2:2:end); 

    vel_min = scenario.v_lim_min;
    A_vel = -Theta_Vel;
    b_vel =  freeResp_Vel- vel_min;

    % concatenate
    A = [A_u;A_vel];
    b = [b_u_max;b_u_min;b_vel];


%% QCQP structure
qp_problem.nCons = length(b) + length(b_coll);
qp_problem.A = A;
qp_problem.A_coll = [A_coll;A_goal];
qp_problem.b_coll = [b_coll;b_goal];
qp_problem.b = b;
qp_problem.ub_du_max = ub_du_max;
qp_problem.lb_du_min = lb_du_min;

qp_problem.p0 = p0;
qp_problem.q0 = q0;
qp_problem.r0 = r0;


end
