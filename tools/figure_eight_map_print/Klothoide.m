function path = Klothoide(a,s,Y0)
% 
% R = 625;
% phi = pi/4;

% a = R * sqrt(2 * phi);
% s = 2 * phi * R;


s_grid = linspace(0,s,1000);
[~,Y] = ode45(@(s_,y_)klode(s_,y_,a), s_grid, Y0);


path = Y(2:end,:);
end

function dY = klode(s,Y,a)
dY = nan(size(Y));
phi = Y(3);
kappa = Y(4);
d_phi = kappa;
d_kappa = 1/a/abs(a);
dx = cos(phi);
dy = sin(phi);
dY(1) = dx;
dY(2) = dy;
dY(3) = d_phi;
dY(4) = d_kappa;
end