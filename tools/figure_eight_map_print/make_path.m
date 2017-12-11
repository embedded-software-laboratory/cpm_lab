function make_path

R = 625;
phi_kl = 0.908453;
a = R * sqrt(2 * phi_kl);
s = 2 * phi_kl * R;

tmp = Klothoide(a, s, [0 0 0 0]);
kl_w = tmp(end,1);
kl_h = tmp(end,2);

phi_circle = pi - 2*phi_kl;
L1 = R-sin(phi_kl)*kl_w+cos(phi_kl)*kl_h;
L_straight_diag = 2*(L1) * sqrt(1+1/(tan(phi_kl)^2));
L_straight = 2*(-kl_w + sin(phi_kl)*R +sin(phi_kl)*kl_h +cos(phi_kl)*kl_w + L1/tan(phi_kl));

L_straight_diag
assert(L_straight_diag>0)
assert(L_straight>0)

path = [0 0 0 0];

path = [path; Klothoide(a, s, path(end,:))];
path = [path; Klothoide(inf, R*phi_circle, path(end,:))];
path = [path; Klothoide(-a, s, path(end,:))];
path = [path; Klothoide(inf, L_straight, path(end,:))];

path = [path; Klothoide(a, s, path(end,:))];
path = [path; Klothoide(inf, R*phi_circle, path(end,:))];
path = [path; Klothoide(-a, s, path(end,:))];
path = [path; Klothoide(inf, L_straight, path(end,:))];

path = [path; Klothoide(a, s, path(end,:))];
path = [path; Klothoide(inf, R*(phi_circle+phi_kl), path(end,:))];
path = [path; Klothoide(-a, s, path(end,:))];
path = [path; Klothoide(inf, L_straight_diag, path(end,:))];
path = [path; Klothoide(-a, s, path(end,:))];
path = [path; Klothoide(inf, R*(phi_circle+2*phi_kl), path(end,:))];
path = [path; Klothoide(a, s, path(end,:))];
path = [path; Klothoide(inf, L_straight_diag, path(end,:))];
path = [path; Klothoide(a, s, path(end,:))];
path = [path; Klothoide(inf, R*phi_circle, path(end,:))];

x = path(:,1);
y = path(:,2);
padding = 100;
x = x - min(x)+padding;
y = y - min(y)+padding;
phi = path(:,3);

c_phi = cos(phi);
s_phi = sin(phi);

track_half_width = 75;

x_l = x + track_half_width * s_phi;
y_l = y - track_half_width * c_phi;

x_r = x - track_half_width * s_phi;
y_r = y + track_half_width * c_phi;


clc
width = max(x)-min(x)+2*padding
height = max(y)-min(y)+2*padding

clf
hold on
plot(x,y)
plot(x_l,y_l)
plot(x_r,y_r)
axis equal
% xlim(xlim+[-1 1]*R)
% ylim(ylim+[-1 1]*R)


end

