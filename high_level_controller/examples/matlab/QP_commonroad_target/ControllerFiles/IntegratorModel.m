%======================================================================
%> @brief Defines vehicle properties as well properties of the dynamics.
%>
%> More detailed description.
%>
%> @retval model Structure with the model properties:
%>  * ode: Function link to the system dgl function
%>  * nx: Amount of state variables
%>  * nu: Amount of input variables
%>  * ny: Amount of output variables
%======================================================================
function [ model ] = model()
    model = struct;
    model.ode = @ode;
    model.A = [0,1,0; 0,0,1; 0,0,-1/0.1];
    model.B = [0;0;1/0.1];
    model.C = [1,0,0; 0,1,0 ];  % outputs are s and v
    model.nx = 3;
    model.nu = 1;
    model.ny = 2;
end

%======================================================================
%> @brief Defines system differential equations parametically.
%>
%> @param x Current state vector:
%> * x1: s-position
%> * x2: velocity
%> * x3: acceleration
%> @param u_ref Reference input values, which the input values shall have,
%> since they are also inert, they have their own ode.
%> 
%> 
%> @retval dx Current differential equations vector of the system
%======================================================================
function dx = ode(x,u_ref)
    dx = x; 
    
    dx(1) =  x(2);
    dx(2) =  x(3);
    dx(3) = (u_ref(1)-x(3))/0.1; % T1 = 0.1 sek
 %   dx(3) = u_ref(1);
end
