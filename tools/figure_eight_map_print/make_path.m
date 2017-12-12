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
phi = path(:,3);

width_mm = 3300;
height_mm = 1700;

line_width_mm = 10;
track_half_width = 75 - line_width_mm/2;
padding_x = (width_mm - (max(x) - min(x)))/2;
padding_y = (height_mm - (max(y) - min(y)))/2;

x = x - min(x) + padding_x;
y = y - min(y) + padding_y;


c_phi = cos(phi);
s_phi = sin(phi);



x_l = x + track_half_width * s_phi;
y_l = y - track_half_width * c_phi;

x_r = x - track_half_width * s_phi;
y_r = y + track_half_width * c_phi;

close all
figure('Visible','off')
hold on


mm_to_pt = 1 / 25.4 * 72;
rectangle('Position',[0 0 width_mm height_mm] * mm_to_pt,'FaceColor',[0 .9 0],'EdgeColor','none')

for dot_x = 100:100:width_mm-50
    for dot_y = 100:100:height_mm-50
        dot_sz_mm = 5;
        rectangle('Position',[(dot_x-dot_sz_mm/2) (dot_y-dot_sz_mm/2) dot_sz_mm dot_sz_mm] * mm_to_pt,'FaceColor',[.9 0 0],'EdgeColor','none','Curvature',1)
    end
end



plot(x_l * mm_to_pt, y_l * mm_to_pt, 'LineWidth', line_width_mm * mm_to_pt, 'Color', 'white')
plot(x_r * mm_to_pt, y_r * mm_to_pt, 'LineWidth', line_width_mm * mm_to_pt, 'Color', 'white')
axis equal
axis off




ax = gca;
ax.Position = [0 0 1 1];
xlim([0 width_mm * mm_to_pt]);
ylim([0 height_mm * mm_to_pt]);

fig = gcf;
fig.PaperPositionMode = 'manual';
fig.PaperType = '<custom>';
fig.PaperUnits = 'points';
fig.PaperSize = [width_mm height_mm] * mm_to_pt;
fig.InvertHardcopy = 'off';
% fig.Color = 'black';
fig.PaperPosition = [0 0 width_mm height_mm] * mm_to_pt;

print(fig, 'x.pdf', '-dpdf', '-painters')

close all

end

