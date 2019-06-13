
data = [ ... % Time(minutes), voltage
                  0       8.25   ; ... % Line 1
                 18       8.05   ; ...
                 40       7.83   ; ...
                 60       7.68   ; ...
                 80       7.55   ; ... 
                101       7.47   ; ... % Line 2
                120       7.42   ; ...
                140       7.35   ; ...
                150       7.27   ; ...
                160       7.22   ; ... 
                164       7.0    ; ... % Line 3
                167       6.8    ; ...
                168       6.5    ; ...
                169       6.4      ...
];

l1_t1 = 0;
l1_t2 = 80;
l1_v1 = 8.2;
l1_v2 = 7.55;
l1_t = l1_t1:l1_t2;
l1_v = (l1_v2-l1_v1) / (l1_t2-l1_t1) * (l1_t-l1_t1) + l1_v1;


l2_t1 = 80;
l2_t2 = 160;
l2_v1 = 7.55;
l2_v2 = 7.22;
l2_t = l2_t1:l2_t2;
l2_v = (l2_v2-l2_v1) / (l2_t2-l2_t1) * (l2_t-l2_t1) + l2_v1;


l3_t1 = 160;
l3_t2 = 169;
l3_v1 = 7.22;
l3_v2 = 6.4;
l3_t = l3_t1:l3_t2;
l3_v = (l3_v2-l3_v1) / (l3_t2-l3_t1) * (l3_t-l3_t1) + l3_v1;

% 7.55 <= V
% l1_battery = @(V) (169-89)/169 / 0.65 * (V-7.55) + 89/169;
l1_battery = @(V)  72.83 * (V-7.55) + 52.66;
% 7.22 <= V < 7.55
l2_battery = @(V) 143.45 * (V-7.22) +  5.33;
% V < 7.22
l3_battery = @(V)   6.49 * (V-6.4 );


figure(1)
clf
box on
plot(data(:,1), data(:,2))
grid on
xlabel('t [min]','FontSize',20)
ylabel('Volt','FontSize',20)

hold on
plot(l1_t, l1_v)
plot(l2_t, l2_v)
plot(l3_t, l3_v)
hold off

figure(2)
clf
box on
max_time = max(data(:,1));
plot(data(:,2), abs(data(:,1)-max_time)/max_time*100)
grid on
xlabel('Volt','FontSize',20)
ylabel('Batterieladung [%]','FontSize',20)

hold on
plot(l1_v, l1_battery(l1_v))
plot(l2_v, l2_battery(l2_v))
plot(l3_v, l3_battery(l3_v))
hold off