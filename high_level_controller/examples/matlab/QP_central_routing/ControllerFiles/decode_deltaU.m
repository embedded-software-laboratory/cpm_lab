% function to transform du into process variables Y
function [Traj,U] = decode_deltaU(scenario,iter,mpc,du_result)

Hp = scenario.Hp;
Hu = scenario.Hu;
nu = scenario.model.nu;
ny = scenario.model.ny;
U = zeros(nu,Hp);
du = reshape(du_result,[nu,Hu]);

% Control values

U(:,1) = du(:,1) + iter.u0;
for k=2:Hu
    U(:,k) = du(:,k) + U(:,k-1);
end
for k=Hu+1:Hp
    U(:,k) = U(:,Hu);
end


% Predicted trajectory

Y = mpc.freeResponse + mpc.Theta*du_result;
Y=reshape(Y,[scenario.model.ny,Hp]);
Traj = Y(1:2,:); % 1:2 corresponds to pos and vel value


