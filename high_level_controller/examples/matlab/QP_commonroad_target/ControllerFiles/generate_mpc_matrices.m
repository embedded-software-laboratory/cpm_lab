
function mpc = generate_mpc_matrices( scenario,iter )

    % Compute all relevant discretization and MPC matrices for all
    % vehicles.
        
    nx = scenario.model.nx;
    nu = scenario.model.nu;
    ny = scenario.model.ny;
    Hp = scenario.Hp;
    Hu = scenario.Hu;
    dt = scenario.dt;

    mpc=struct;
    
    %% GENARATE MATRICES
    for i=1:Hp
       mpc.Reference( blk(i,ny),1 ) = iter.RefData.ReferenceTrajectoryPoints(:,i);
    end
    
    mpc.A = zeros(nx,nx,Hp);
    mpc.B = zeros(nx,nu,Hp);
    mpc.E = zeros(nx,Hp);
    
    [ A,B,C,E ] ...
        = discretize( iter.x0(:), iter.u0(:), dt, scenario.model );
    [ Psi, Gamma, mpc.Theta, Pie ] ...
        = prediction_matrices( A,B,C,nx,nu,ny,Hp,Hu );
    
    mpc.freeResponse = Psi *iter.x0(:) + Gamma *iter.u0(:) + Pie*E;
    
    [ mpc.H , mpc.g, mpc.r ] ...
        = mpc_cost_function_matrices( scenario.Q, scenario.R, nu,ny,Hp,Hu,mpc,scenario.Q_final );
    
    % For the simple linearization, the same A,B,E are used in
    % every prediction step.
    for i=1:Hp
        mpc.A(:,:,i) = A;
        mpc.B(:,:,i) = B;
        mpc.E(:,i) = E;
    end
