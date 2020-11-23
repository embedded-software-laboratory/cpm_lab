

function [U,trajectoryPrediction,controllerOutput] = QP_controller( scenario,iter,prevOutput ) 
    controllerOutput = struct;

    mpc = generate_mpc_matrices( scenario,iter ); 
    qp = convert_to_QP( scenario,iter,mpc );

    % take previous output as initialization
    du_prev = prevOutput.du;
    n_du = length(du_prev);
    du = reshape(du_prev,[n_du,1]);
    du(1:end-1,:)=du(2:end,:);
    du(end,:) = 0;

    
    controllerOutput.resultInvalid = false;

    
    disp('entering optimizer');
    % try last MPC sol.
    [du , feasible , ~, controllerOutput.optimization_log] = QP_optimizer( scenario,iter,qp, mpc, du );
   disp('finished optimization');
    
    if ~feasible
            disp(['\n\n\n']);
    end
    
    controllerOutput.du=du;
    
    controllerOutput.feasible=feasible;
    
    [trajectoryPrediction,U] = decode_deltaU(scenario,iter,mpc,du);

end